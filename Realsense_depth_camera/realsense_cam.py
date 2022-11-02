import numpy as np
import pyrealsense2 as rs
import os, cv2, glob, time, joblib, serial


class realsense_cam:
    def __init__(self, serial_num, img_size=(640, 480), intrinsic_path=''):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial_num)
        config.enable_stream(rs.stream.depth, img_size[0], img_size[1], rs.format.z16, 60)
        config.enable_stream(rs.stream.color, img_size[0], img_size[1], rs.format.bgr8, 60)
        self.profile = self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        if len(intrinsic_path) != 0:
            self.intrinsic_mat, self.dist_coeffs = joblib.load(intrinsic_path)
            fx, fy, ox, oy = self.intrinsic_mat[0, 0], self.intrinsic_mat[1, 1], self.intrinsic_mat[0, 2], self.intrinsic_mat[1, 2]
            self.intrinsic_at = (fx, fy, ox, oy)
        else:
            self.intrinsic_at, self.intrinsic_mat = self.get_intrinsics()

    def get_image(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        color_img = np.asanyarray(color_frame.get_data())
        depth_img = np.asanyarray(depth_frame.get_data())

        return color_img, depth_img

    def get_intrinsics(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        fx, fy, ox, oy = color_intrinsics.fx, color_intrinsics.fy, color_intrinsics.ppx, color_intrinsics.ppy

        return (fx, fy, ox, oy), np.array([[fx, 0, ox], [0, fy, oy], [0, 0, 1]])

    def enable_InfraRed(self, enable=True):
        IR_range = self.depth_sensor.get_option_range(rs.option.laser_power)

        if enable:
            self.depth_sensor.set_option(rs.option.laser_power, IR_range.max)
            time.sleep(0.5)
        else:
            self.depth_sensor.set_option(rs.option.laser_power, IR_range.min)

    def process_end(self):
        self.pipeline.stop()


def get_realsense_serial_num():
    serial_num = []
    ctx = rs.context()
    if len(ctx.devices) > 0:
        for d in ctx.devices:
            serial_num.append(d.get_info(rs.camera_info.serial_number))
            print(f'Found device: {d.get_info(rs.camera_info.name)} '
                  f'(Serial Num: {d.get_info(rs.camera_info.serial_number)})')
    else:
        print('No Intel Device connected.')

    return serial_num


def cam_calibration(cam, img_size=(640, 480), board_size=(8, 6), target_path='./calibration/cam/'):
    color_path = target_path + 'color/'
    depth_path = target_path + 'depth/'
    os.makedirs(color_path, exist_ok=True)
    os.makedirs(depth_path, exist_ok=True)

    # take imgs of the chessboard
    cam.enable_InfraRed(False)
    i = 0
    while True:
        color_img, _ = cam.get_image()
        cv2.imshow('Current View', color_img)
        return_char = cv2.waitKey(1) & 0xFF
        if return_char == 27:
            break
        elif return_char == ord('p'):
            cv2.imwrite(f'{color_path}{i}.jpg', color_img)
            print(f'{color_path}{i}.jpg saved.')
            i += 1

    # camera calibration
    corner_x, corner_y = board_size[0] - 1, board_size[1] - 1
    objp = np.zeros((corner_x * corner_y, 3), np.float32)
    objp[:, :2] = np.mgrid[0:corner_x, 0:corner_y].T.reshape(-1, 2)
    obj_pts = []  # 3d points in real world space
    img_pts = []  # 2d points in image plane.
    imgs = glob.glob(f'{color_path}*.jpg')

    for fname in imgs:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        print('find the chessboard corners of', fname)
        ret, corners = cv2.findChessboardCorners(gray, (corner_x, corner_y), None)

        # If found, add object points, image points
        if ret == True:
            obj_pts.append(objp)
            img_pts.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (corner_x, corner_y), corners, ret)
            cv2.imshow('chess board', img)

    print('Camera calibration...')
    _, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(obj_pts, img_pts, img_size, None, None)
    joblib.dump((camera_matrix, dist_coeffs), f'{color_path}intrinsics.pkl')
    print(f'{color_path}intrinsics.pkl saved.')


if __name__ == '__main__':
    # make target dir
    img_path = './data/'
    os.makedirs(img_path, exist_ok=True)

    # get camera serial number
    get_realsense_serial_num()
    serial_num_top = '639207002295'
    serial_num_beside = '635206006429'
    serial_num_robot = '635202000911'

    # initialize camera
    cam_top = realsense_cam(serial_num_top)
    cam_beside = realsense_cam(serial_num_beside)
    cam_robot = realsense_cam(serial_num_robot)

    # calibrate camera intrinsics
    cam_calibration(cam_top)
    cam_calibration(cam_beside)
    cam_calibration(cam_robot)

    # enable & disable camera IR
    cam_top.enable_InfraRed()
    cam_beside.enable_InfraRed(enable=False)
    cam_robot.enable_InfraRed(enable=False)

    # start capturing imgs
    video_writer = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 50.0, (640, 480))
    frame_id = 0
    while True:
        # get color & depth imgs
        color_top, depth_top = cam_top.get_image()
        color_beside, _ = cam_beside.get_image()
        color_robot, _ = cam_robot.get_image()

        # leave
        wKey = cv2.waitKey(1) & 0xFF
        if wKey == 27:
            break

        # capture single img
        if wKey == ord('c'):
            print(f'{img_path}{frame_id}.color.jpg')
            cv2.imwrite(f'{img_path}{frame_id}.color.jpg', c)
            frame_id += 1

        # show & write to video
        cur_view = np.hstack((color_top, color_beside, color_robot))
        cv2.imshow('Current View', cur_view)
        video_writer.write(cur_view)

    # end process
    cam_top.process_end()
    cam_beside.process_end()
    cam_robot.process_end()
    cv2.destroyAllWindows()


