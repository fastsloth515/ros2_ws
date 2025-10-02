#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import numpy as np
import cv2
import pycuda.autoinit
import pycuda.gpuarray as gpuarray
from pycuda.compiler import SourceModule
from rclpy.qos import qos_profile_sensor_data

KERNEL_CODE = """
__global__ void bev_kernel(
    float *x, float *y, float *z,
    unsigned char *r, unsigned char *g, unsigned char *b,
    int num_points, int width, int height, float res,
    unsigned char *bev_img)
{
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= num_points) return;

    // BEV 평면: X (좌우), Z (전방)
    int gx = (int)(x[idx] / res + width / 2);
    int gz = (int)(height - z[idx] / res);

    if (gx >= 0 && gx < width && gz >= 0 && gz < height) {
        // dilation: 반경 3픽셀(7x7)
        for (int dx=-1; dx<=1; dx++) {
            for (int dy=-1; dy<=1; dy++) {
                int nx = gx + dx;
                int ny = gz + dy;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int offset = (ny * width + nx) * 3;
                    bev_img[offset + 0] = b[idx];
                    bev_img[offset + 1] = g[idx];
                    bev_img[offset + 2] = r[idx];
                }
            }
        }
    }
}
"""

class BEVGridNode(Node):
    def __init__(self):
        super().__init__('bev_grid_node')

        # Subscribers
        self.create_subscription(
            PointCloud2,
            '/camera/depth_registered/points',
            self.pc_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self.pub_img = self.create_publisher(Image, '/bev/image', 10)
        self.pub_occ = self.create_publisher(OccupancyGrid, '/bev/occupancy_grid', 10)

        self.bridge = CvBridge()

        # Grid parameters
        self.res = 0.05  # 1 cm per cell
        self.width, self.height = 150,100  # 5m x 5m grid
        self.origin_x = - (self.width * self.res) / 2.0 
        self.origin_y = - (self.height * self.res) / 2.0

        # Camera offset (base_footprint 기준)
        self.dx = 0.34  # 전방 오프셋
        self.dy = 0.0   # 좌우 오프셋
        self.originx = -1.0 #그리드 x축 시작점

        # CUDA kernel 준비
        self.mod = SourceModule(KERNEL_CODE)
        self.kernel = self.mod.get_function("bev_kernel")

    def pc_callback(self, msg):
        # --- (1) PointCloud2 raw data에서 x,y,z좌표, RGB 값 추출 -> numpy ---
        cloud_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        if cloud_arr.shape[0] == 0:
            return

        x = np.frombuffer(cloud_arr[:,0:4].tobytes(), dtype=np.float32)
        y = np.frombuffer(cloud_arr[:,4:8].tobytes(), dtype=np.float32)
        z = np.frombuffer(cloud_arr[:,8:12].tobytes(), dtype=np.float32)
        rgb_float = np.frombuffer(cloud_arr[:,16:20].tobytes(), dtype=np.float32)

        rgb_int = rgb_float.view(np.uint32)
        r = ((rgb_int >> 16) & 0xFF).astype(np.uint8)
        g = ((rgb_int >> 8) & 0xFF).astype(np.uint8)
        b = (rgb_int & 0xFF).astype(np.uint8)

        num_points = len(x)

        # --- (2) CUDA kernel 실행: BEV 이미지 생성 ---
        # 데이터 gpu로 전송 후에 CUDA 커널 실행
        x_gpu = gpuarray.to_gpu(x)
        y_gpu = gpuarray.to_gpu(y)
        z_gpu = gpuarray.to_gpu(z)
        r_gpu = gpuarray.to_gpu(r)
        g_gpu = gpuarray.to_gpu(g)
        b_gpu = gpuarray.to_gpu(b)
        bev_gpu = gpuarray.zeros((self.height * self.width * 3), np.uint8)

        block = (256, 1, 1)
        grid = ((num_points + 255)//256, 1)
        self.kernel(x_gpu, y_gpu, z_gpu,
                    r_gpu, g_gpu, b_gpu,
                    np.int32(num_points),
                    np.int32(self.width), np.int32(self.height),
                    np.float32(self.res), bev_gpu,
                    block=block, grid=grid)

        bev = bev_gpu.get().reshape((self.height, self.width, 3))

        # --- (3) BEV 이미지를 퍼블리시 ---
        img_msg = self.bridge.cv2_to_imgmsg(bev, encoding="bgr8")
        self.pub_img.publish(img_msg)

        # --- (4) OccupancyGrid 생성 ---
        occ = OccupancyGrid()
        occ.header.stamp = self.get_clock().now().to_msg()
        occ.header.frame_id = "map"

        occ.info.resolution = self.res
        occ.info.width = self.width
        occ.info.height = self.height
        occ.info.origin.position.x = self.originx 
        occ.info.origin.position.y = self.origin_y
        occ.info.origin.orientation.w = 1.0

        # === BEV → HSV 변환 ===
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)

        # 색상 마스크 정의
        # 빨강, 초록, 파랑 픽셀 각자 추출 -> 도달 가능 영역 분류 위함
        lower_green = np.array([40, 70, 70]); upper_green = np.array([80, 255, 255])
        lower_red1  = np.array([0, 120, 70]); upper_red1  = np.array([10, 255, 255])
        lower_red2  = np.array([170, 120, 70]); upper_red2 = np.array([180, 255, 255])
        lower_blue  = np.array([100, 150, 70]); upper_blue = np.array([140, 255, 255])
        lower_white = np.array([0, 0, 200]); upper_white = np.array([180, 40, 255])

        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_red   = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        mask_blue  = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_white  = cv2.inRange(hsv, lower_white, upper_white)
        mask_avoid = (mask_red > 0) | (mask_white > 0)



        # === (1) 픽셀 좌표 그리드 ===
        jj, ii = np.meshgrid(np.arange(self.width), np.arange(self.height))

        # === (2) 픽셀 → BEV 좌표 (x_bev, z_bev) ===
        x_bev = (jj - self.width/2) * self.res   # 좌우
        z_bev = (self.height/2 - ii) * self.res  # 전방

        # === (3) BEV → base_footprint (x_fp, y_fp) ===
        x_fp = z_bev + self.dx   # 전방 → X
        y_fp = -x_bev + self.dy   # 좌우 → Y

        # === (4) base_footprint → OccupancyGrid 인덱스 ===
        j_grid = ((x_fp - self.origin_x) / self.res).astype(np.int32)
        i_grid = ((y_fp - self.origin_y) / self.res).astype(np.int32)

        valid_mask = (i_grid >= 0) & (i_grid < self.height) & \
                     (j_grid >= 0) & (j_grid < self.width)

        # === (5) OccupancyGrid 데이터 초기화 ===
        #  -1 : unknown, 0 : free space(green), 70 : 회피 구역(red), 100 : 점유/장애물(blue)
        grid_np = np.full((self.height, self.width), -1, dtype=np.int8)

        grid_np[i_grid[(mask_blue > 0) & valid_mask],
                j_grid[(mask_blue > 0) & valid_mask]] = 100
        grid_np[i_grid[(mask_avoid > 0) & valid_mask],
                j_grid[(mask_avoid > 0) & valid_mask]] = 70
        grid_np[i_grid[(mask_green > 0) & valid_mask],
                j_grid[(mask_green > 0) & valid_mask]] = 0

        # === (6) Dilation (옵션) ===
        kernel = np.ones((3,3), np.uint8)
        obstacle_mask = (grid_np == 100).astype(np.uint8)
        avoid_mask    = (grid_np == 70).astype(np.uint8)

        obstacle_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)
        avoid_mask    = cv2.dilate(avoid_mask, kernel, iterations=1)

        grid_np[obstacle_mask > 0] = 100
        grid_np[avoid_mask > 0]    = 70

        # === (7) OccupancyGrid 메시지 채우기 ===
        occ.data = grid_np.flatten().tolist()
        self.pub_occ.publish(occ)

        self.get_logger().info(f"Published BEV + OccupancyGrid | Points: {num_points}")


def main(args=None):
    rclpy.init(args=args)
    node = BEVGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
