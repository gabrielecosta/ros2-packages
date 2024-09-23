import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import cv2
import numpy as np
import glob

class CalibrationServer(Node):
    def __init__(self):
        super().__init__('calibration_server')
        self.srv = self.create_service(Trigger, 'calibrate_camera', self.calibrate_camera_callback)
        # creo anche qui un servizio server abilitato da una richiesta trigger
        # Specify the images and parameters directories
        # definisco le cartelle dove salvare i parametri
        self.images_folder = 'images'
        self.parameters_folder = 'parameters'
        os.makedirs(self.parameters_folder, exist_ok=True)
        self.calibration_params_file = os.path.join(self.parameters_folder, 'calibration_params.npz')

    def calibrate_camera_callback(self, request, response):
        '''
        Questa callback viene richiamata quando sopraggiunge una richiesta trigger al server.
        Quindi, inizia il processo di calibrazione e di salvataggio dei parametri,
        nonché di responso positivo della calibrazione stessa
        '''
        # Collect the image paths
        image_files = glob.glob(f'{self.images_folder}/*.png')
        print(image_files)
        if len(image_files) < 10:
            response.success = False
            response.message = 'Not enough images for calibration.'
            return response

        # Calibration process
        ret, camera_matrix, dist_coeffs = self.perform_calibration(image_files)
        if ret:
            # salvataggio dati
            np.savez(self.calibration_params_file, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
            self.get_logger().info('Camera calibrated successfully.')
            self.get_logger().info(f'Saved parameters to {self.calibration_params_file}')
            response.success = True
            response.message = 'Calibration successful.'
        else:
            response.success = False
            response.message = 'Calibration failed.'

        return response

    def perform_calibration(self, image_files):
        # Define the dimensions of the checkerboard
        CHECKERBOARD = (9, 6)  # The number of inner corners per chessboard row and column
        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Create vectors to store object points and image points from all the images
        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane

        # Define the real world coordinates for points in the checkerboard
        objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

        # Adjust the path to your directory
        images = glob.glob('images/*.png')

        for fname in image_files:
            print(f'Opened {fname}')
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

            if ret:
                objpoints.append(objp)
                imgpoints.append(corners)

        # Perform camera calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        return ret, camera_matrix, dist_coeffs


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
