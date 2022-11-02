import os, cv2, sys, glob, joblib
import numpy as np
import transforms3d as tf3d
import math3d as m3d
from datetime import datetime

from handeye_calib.dual_quaternion import DualQuaternion as dq
from handeye_calib.dual_quaternion_hand_eye_calibration import HandEyeConfig, \
    compute_hand_eye_calibration_RANSAC, compute_hand_eye_calibration_BASELINE
import handeye_calib.transformations as trf

from realsense_cam import realsense_cam
from utils.utils import solve_pose, print_m3d_pose
from utils.minisam_optim import opt_poses

# robot
sys.path.append(os.path.abspath(os.path.join('..', 'UR10_robot_operation')))
from UR10_robot_operation.ur10_robot import UR10Robot

# apriltag
sys.path.append(os.path.abspath(os.path.join('..', 'Apriltag')))
from Apriltag.tag_detection import tag_boards, detect_tags, get_tagboard_obj_pts


def cam_calibration(cam, img_size=(640, 480), board_size=(8, 6), square_size=0.031,
                    target_path='./calibration/cam_calib/', calib_robot_pose_path='calib_ur10Ttcp_seq.pkl'):
    # info
    color_path = target_path + 'color/'
    os.makedirs(color_path, exist_ok=True)

    # initial robot
    try:
        ur10Ttcp_seq = joblib.load(calib_robot_pose_path)
        print(f'{calib_robot_pose_path} loaded.')
    except:
        ur10Ttcp_seq = []
    saved_poses = (len(ur10Ttcp_seq) != 0)
    robot = UR10Robot()

    # take imgs of the chessboard
    cam.enable_InfraRed(False)
    img_i = 0
    pose_i = 0
    while True:
        color_img, _ = cam.get_image()
        cv2.imshow('Current View', color_img)
        return_char = cv2.waitKey(1) & 0xFF
        if return_char == 27:
            robot.close()
            break
        elif return_char == ord('r') and saved_poses:
            robot.moveL(ur10Ttcp_seq[pose_i])
            pose_i += 1
        elif return_char == ord('s'):
            cv2.imwrite(f'{color_path}{img_i}.jpg', color_img)
            print(f'{color_path}{img_i}.jpg saved.')
            if not saved_poses:
                ur10Ttcp_i = robot.get_pose()
                ur10Ttcp_seq.append(ur10Ttcp_i)
                joblib.dump(ur10Ttcp_seq, calib_robot_pose_path)
                print(f'{calib_robot_pose_path} saved.')
            img_i += 1

    # camera calibration
    corner_x, corner_y = board_size[0] - 1, board_size[1] - 1
    objp = np.zeros((corner_x * corner_y, 3), np.float32)
    objp[:, :2] = np.mgrid[0:corner_x, 0:corner_y].T.reshape(-1, 2) * square_size
    obj_pts = []  # 3d points in real world space
    img_pts = []  # 2d points in image plane.
    imgs = glob.glob(f'{color_path}*.jpg')

    for fname in imgs:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        print('find the chessboard corners of', fname)
        ret, corners = cv2.findChessboardCorners(gray, (corner_x, corner_y), flags=cv2.CALIB_CB_ACCURACY)

        # If found, add object points, image points
        if ret:
            # Ensure that all corner-arrays are going from top to bottom.
            # ref: https://github.com/ros-perception/image_pipeline/blob/noetic/camera_calibration/src/camera_calibration/calibrator.py#L214
            if corners[0, 0, 1] > corners[-1, 0, 1]:
                corners = np.copy(np.flipud(corners))

            obj_pts.append(objp)
            img_pts.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (corner_x, corner_y), corners, ret)
            cv2.imshow('chess board', img)

    print('Camera calibration...')
    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(obj_pts, img_pts, img_size, None, None)
    joblib.dump((camera_matrix, dist_coeffs), f'{target_path}intrinsics.pkl')
    print(f'{target_path}intrinsics.pkl saved.')
    print("Calibrated: rms={:5f}".format(rms))
    print('camera_matrix: ', camera_matrix)
    print('dist_coeffs: ', dist_coeffs)
    # A good result should have RMS lower than 0.5
    if rms > 0.7:
        print('Please do the calibration again')

    # # depth calibration
    # cam.enable_InfraRed(True)


def multicam_calib(cam1, cam2, board_name='board4x6', target_path='./calibration/multicam/', T_name='cam1Tcam2'):
    os.makedirs(target_path, exist_ok=True)

    cam1Tcam2_seq, i = [], 1
    tagboard_dict, tag_size = tag_boards(board_name)
    while True:
        # get img
        color_img1, _ = cam1.get_image()
        color_img2, _ = cam2.get_image()

        # detect tags
        detect_img1, tag_IDs1, tag_img_pts1 = detect_tags(color_img1, cam1.intrinsic_at, tag_size)
        detect_img2, tag_IDs2, tag_img_pts2 = detect_tags(color_img2, cam2.intrinsic_at, tag_size)
        detect_imgs = np.hstack([detect_img1, detect_img2])
        cv2.imshow('multicam_calib', detect_imgs)

        return_char = cv2.waitKey(50)
        if return_char & 0xFF == ord('s') and len(tag_IDs1) > 10 and len(tag_IDs2) > 10:
            tag_img_pts1 = np.array(tag_img_pts1).reshape(-1, 2)
            tag_img_pts2 = np.array(tag_img_pts2).reshape(-1, 2)
            tag_obj_pts1 = get_tagboard_obj_pts(tagboard_dict, tag_IDs1)
            tag_obj_pts2 = get_tagboard_obj_pts(tagboard_dict, tag_IDs2)

            m3d_transform1 = solve_pose(tag_obj_pts1, tag_img_pts1, cam1.intrinsic_mat)
            m3d_transform2 = solve_pose(tag_obj_pts2, tag_img_pts2, cam2.intrinsic_mat)
            cam1Tcam2 = m3d_transform1 * m3d_transform2.inverse
            print_m3d_pose(cam1Tcam2)
            cam1Tcam2_seq.append(cam1Tcam2)
            print('Pose {} saved'.format(i))
            i += 1
        elif return_char & 0xFF == 27:  # esc
            cam1.process_end()
            cam2.process_end()
            cv2.destroyAllWindows()
            break

    optim_cam1Tcam2 = opt_poses(cam1Tcam2_seq)
    print_m3d_pose(optim_cam1Tcam2, f'Optimal {T_name} pose')

    joblib.dump(cam1Tcam2_seq, f'{target_path}{T_name}_seq.pkl')
    joblib.dump(optim_cam1Tcam2, f'{target_path}optim_{T_name}.pkl')
    print(f'{target_path}{T_name}_seq.pkl saved.')
    print(f'{target_path}optim_{T_name}.pkl saved.')


# Pose form: x, y, z, q_x, q_y, q_z, q_w
def solve_relative_transformation(base_c1, base_c2):
    # Usage 1. base_T_hand,world_T_eye (eye on hand) ->Return: handTeye[baseline/RANSAC]
    # Usage 2. world_T_cam1,world_T_cam2 -> cam1Tcam2 ->Return: cam1Tcam2[baseline/RANSAC]
    conf = HandEyeConfig()
    conf.min_num_inliers = len(base_c1) // 1.2
    conf.prefilter_poses_enabled = False
    conf.enable_exhaustive_search = True
    pose_base_hand = [dq.from_pose_vector(p) for p in base_c1]  # Hand in robot-base frame
    pose_world_eye = [dq.from_pose_vector(p) for p in base_c2]  # Eye in world frame
    result_baseline = compute_hand_eye_calibration_BASELINE(pose_base_hand, pose_world_eye, conf)
    # result_RANSAC = compute_hand_eye_calibration_RANSAC(pose_base_hand, pose_world_eye, conf)
    quat_bl = result_baseline[1].to_pose()
    # quat_rs = result_RANSAC[1].to_pose()
    # x, y, z, q_x, q_y, q_z, q_w
    c1Tc2_bl = trf.quaternion_matrix([quat_bl[6], quat_bl[3], quat_bl[4], quat_bl[5]])  # w,x,y,z
    c1Tc2_bl[0:3, 3] = quat_bl[0:3]
    # c1Tc2_rs = trf.quaternion_matrix([quat_rs[6], quat_rs[3], quat_rs[4], quat_rs[5]])  # w,x,y,z
    # c1Tc2_rs[0:3, 3] = quat_rs[0:3]
    return c1Tc2_bl  # , c1Tc2_rs


def handeye_calib_ur10(cam, board_name='board4x6',
                       target_path='./calibration/handeye_calib_ur10/',
                       handTeye_name='handTeye', baseThand_name='baseThand', worldTeye_name='worldTeye'):
    # baseThand: ur10Ttcp
    # worldTeye: tagboardTcam2
    # handTeye: tcpTcam2

    # baseThand: ur10Ttcp
    # worldTeye: cam3Tgrip
    # handTeye: tcpTgrip

    os.makedirs(target_path, exist_ok=True)

    robot = UR10Robot()
    origin_ur10Ttcp_seq = []
    baseThand_seq, worldTeye_seq = [], []
    i = len(baseThand_seq) + 1
    tagboard_dict, tag_size = tag_boards(board_name)

    while True:
        color_img, _ = cam.get_image()
        detect_img, tag_IDs, tag_img_pts = detect_tags(color_img, cam.intrinsic_at, tag_size)
        tag_obj_pts = get_tagboard_obj_pts(tagboard_dict, tag_IDs)
        cv2.imshow('handeye_calib_ur10', detect_img)

        return_char = cv2.waitKey(50)
        if return_char & 0xFF == ord('s') and len(tag_IDs) > 10:
            ur10Ttcp_i = robot.get_pose()
            origin_ur10Ttcp_seq.append(ur10Ttcp_i)
            quad_bTh = tf3d.quaternions.mat2quat(ur10Ttcp_i.orient.array)  # qw,qx,qy,qz
            quad_bTh = quad_bTh[[1, 2, 3, 0]].tolist()  # qx,qy,qz,qw
            quad_bTh = ur10Ttcp_i.pos.array.tolist() + quad_bTh
            baseThand_seq.append(np.array(quad_bTh))

            worldTeye_i = solve_pose(tag_obj_pts, np.array(tag_img_pts).reshape(-1, 2), cam.intrinsic_mat).inverse
            quad_wTcam = tf3d.quaternions.mat2quat(worldTeye_i.orient.array)
            quad_wTcam = quad_wTcam[[1, 2, 3, 0]].tolist()  # qx,qy,qz,qw
            quad_wTcam = worldTeye_i.pos.array.tolist() + quad_wTcam
            worldTeye_seq.append(np.array(quad_wTcam))

            print('Pose {} saved'.format(i))
            joblib.dump(origin_ur10Ttcp_seq, f'{target_path}origin_ur10Ttcp_seq.pkl')
            joblib.dump(baseThand_seq, f'{target_path}{baseThand_name}_seq.pkl')
            joblib.dump(worldTeye_seq, f'{target_path}{worldTeye_name}_seq.pkl')
            print(f'{target_path}origin_ur10Ttcp_seq.pkl saved.')
            print(f'{target_path}{baseThand_name}_seq.pkl saved.')
            print(f'{target_path}{worldTeye_name}_seq.pkl saved.')
            i += 1
        elif return_char & 0xFF == 27:  # esc
            cam.process_end()
            robot.close()
            cv2.destroyAllWindows()
            break

    handTeye = solve_relative_transformation(np.vstack(baseThand_seq), np.vstack(worldTeye_seq))
    optim_handTeye = m3d.Transform(handTeye)
    print_m3d_pose(optim_handTeye, f'Optimal {handTeye_name} pose')

    # dump pose (for future use)
    joblib.dump(optim_handTeye, f'{target_path}optim_{handTeye_name}.pkl')
    joblib.dump(baseThand_seq, f'{target_path}{baseThand_name}_seq.pkl')
    joblib.dump(worldTeye_seq, f'{target_path}{worldTeye_name}_seq.pkl')
    print(f'{target_path}optim_{handTeye_name}.pkl saved.')
    print(f'{target_path}{baseThand_name}_seq.pkl saved.')
    print(f'{target_path}{worldTeye_name}_seq.pkl saved.')


def solve_transform(target_path='./calibration/handeye_calib_ur10/',
                    handTeye_name='handTeye', baseThand_name='baseThand', worldTeye_name='worldTeye'):
    baseThand_seq = joblib.load(f'{target_path}{baseThand_name}_seq.pkl')
    worldTeye_seq = joblib.load(f'{target_path}{worldTeye_name}_seq.pkl')

    handTeye = solve_relative_transformation(np.vstack(baseThand_seq), np.vstack(worldTeye_seq))
    optim_handTeye = m3d.Transform(handTeye)
    print_m3d_pose(optim_handTeye, f'Optimal {handTeye_name} pose')

    # dump pose (for future use)
    joblib.dump(optim_handTeye, f'{target_path}optim_{handTeye_name}.pkl')
    joblib.dump(baseThand_seq, f'{target_path}{baseThand_name}_seq.pkl')
    joblib.dump(worldTeye_seq, f'{target_path}{worldTeye_name}_seq.pkl')
    print(f'{target_path}optim_{handTeye_name}.pkl saved.')
    print(f'{target_path}{baseThand_name}_seq.pkl saved.')
    print(f'{target_path}{worldTeye_name}_seq.pkl saved.')



if __name__ == '__main__':
    # initialize cameras
    serial_num_top = '639207002295'
    serial_num_beside = '635206006429'
    serial_num_robot = '635202000911'
    cam_top = realsense_cam(serial_num=serial_num_top, intrinsic_path=f'./cam_intrinsics/intrinsics_{serial_num_top}.pkl')
    cam_beside = realsense_cam(serial_num_beside)
    cam_robot = realsense_cam(serial_num_robot)
    cam_top.enable_InfraRed()
    cam_beside.enable_InfraRed()
    cam_robot.enable_InfraRed()

    # camera intrinsic calibration
    cam_calibration(cam_top, target_path=f'./calibration{datetime.now().strftime("%Y%m%d_%H%M%S")}/cam_calib/')

    # handeye calibration: tcpTcam_robot
    handeye_calib_ur10(cam_robot,
                       handTeye_name='tcpTcam_robot', baseThand_name='ur10Ttcp', worldTeye_name='tagboardTcam_robot')
    # Optimal tcpTcam_robot pose: rx, ry, rz, x, y, z

    # multi-camera calibration: cam_topTcam_robot
    robot = UR10Robot()
    robot.restore_poses('./calibration/cam_topTcam_robot_pose')
    robot.set_pose_by_key('home')
    robot.close()
    multicam_calib(cam_top, cam_robot, T_name='cam_topTcam_robot')

    # multi-camera calibration: cam_besideTcam_top
    multicam_calib(cam_beside, cam_top, T_name='cam_besideTcam_top')
