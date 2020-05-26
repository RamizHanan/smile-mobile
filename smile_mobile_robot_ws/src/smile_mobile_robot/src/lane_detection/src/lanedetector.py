import numpy as np
import pandas as pd
import math
import os
import matplotlib.pyplot as plt
import cv2


class LaneDetector():

    def __init__(self):
        self.left_a, self.left_b, self.left_c = [], [], []
        self.right_a, self.right_b, self.right_c = [], [], []

    def filter(self, img, s_thresh=(100, 255), sx_thresh=(15, 255)):
        '''
        An HSV filter that filters out the color blue from the input image and
        returns a black and white image.

        :param img: A source image that will be filtered

        :return np.ndarray: An image containing the filtered color only
        '''

        # img = np.copy(img)
        # # Convert to HLS color space and separate the V channel
        # hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)
        # l_channel = hls[:, :, 1]
        # s_channel = hls[:, :, 2]
        # h_channel = hls[:, :, 0]
        # # Sobel x
        # sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 1)  # Take the derivative in x
        # abs_sobelx = np.absolute(sobelx)  # Absolute x derivative to accentuate lines away from horizontal
        # scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
        # # Threshold x gradient
        # sxbinary = np.zeros_like(scaled_sobel)
        # sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
        # # Threshold color channel
        # s_binary = np.zeros_like(s_channel)
        # s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
        # color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary)) * 255
        # combined_binary = np.zeros_like(sxbinary)
        # combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1
        # return combined_binary

        # Convert RGB to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # define range of blue color in HSV
        # lower_blue = np.array([100, 0, 0])
        # upper_blue = np.array([130, 255, 255])

        # define range of white color in HSV
        # lower_white = np.array([0,0,0])
        # upper_white = np.array([0,0,255])

        # define range of gray color in HSV
        lower_gray = np.array([0,0,40])
        upper_gray = np.array([360,20,180])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_gray, upper_gray)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img, img, mask=mask)
        _, res = cv2.threshold(res, 0, 255, cv2.THRESH_BINARY)
        return res[:, :, 0]

    def perspective_warp(self, img, dst_size=(640, 480),
                        src=np.float32([(0.35, 0.44), (0.67, 0.44), (0.07, 0.72), (.94, 0.72)]),
                        dst=np.float32([(0, 0), (1, 0), (0, 1), (1, 1)])):

        # src: (x,y) -> TopLeft, TopRight, BottomLeft, BottomRight
        img_size = np.float32([(img.shape[1], img.shape[0])])
        src = src * img_size
        dst = dst * np.float32(dst_size)
        # Given src and dst points, calculate the perspective transform matrix
        M = cv2.getPerspectiveTransform(src, dst)
        # Warp the image usingx warpPerspective()
        warped = cv2.warpPerspective(img, M, dst_size)
        return warped

    def inv_perspective_warp(self, img,
                            dst_size=(640, 480),
                            src=np.float32([(0, 0), (1, 0), (0, 1), (1, 1)]),
                            dst=np.float32([(0.35, 0.44), (0.67, 0.44), (0.07, 0.72), (.94, 0.72)])):
        img_size = np.float32([(img.shape[1], img.shape[0])])
        src = src * img_size
        dst = dst * np.float32(dst_size)
        # Given src and dst points, calculate the perspective transform matrix
        M = cv2.getPerspectiveTransform(src, dst)
        # Warp the image using warpPerspective()
        warped = cv2.warpPerspective(img, M, dst_size)
        return warped

    def get_hist(self, img):
        '''
        A method that returns a sum of pixels along the x axis of an image
        :param img: An image that will be used to find pixels
        :return list: A list containing the values along the x axis
        '''
        hist = np.sum(img[img.shape[0]//2:, :], axis=0)
        return hist

    def sliding_window(self, img, nwindows=9, margin=150, minpix=1, draw_windows=True):
        left_fit_ = np.empty(3)
        right_fit_ = np.empty(3)
        out_img = np.dstack((img, img, img))*255

        histogram = self.get_hist(img)
        # find peaks of left and right halves
        midpoint = int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        # Set height of windows
        window_height = np.int(img.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img.shape[0] - (window+1)*window_height
            win_y_high = img.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            if draw_windows == True:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
                (100, 255, 255), 3)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high),
                (100, 255, 255), 3)
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
    #        if len(good_right_inds) > minpix:        
    #            rightx_current = np.int(np.mean([leftx_current +900, np.mean(nonzerox[good_right_inds])]))
    #        elif len(good_left_inds) > minpix:
    #            rightx_current = np.int(np.mean([np.mean(nonzerox[good_left_inds]) +900, rightx_current]))
    #        if len(good_left_inds) > minpix:
    #            leftx_current = np.int(np.mean([rightx_current -900, np.mean(nonzerox[good_left_inds])]))
    #        elif len(good_right_inds) > minpix:
    #            leftx_current = np.int(np.mean([np.mean(nonzerox[good_right_inds]) -900, leftx_current]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 

        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        self.left_a.append(left_fit[0])
        self.left_b.append(left_fit[1])
        self.left_c.append(left_fit[2])

        self.right_a.append(right_fit[0])
        self.right_b.append(right_fit[1])
        self.right_c.append(right_fit[2])

        left_fit_[0] = np.mean(self.left_a[-10:])
        left_fit_[1] = np.mean(self.left_b[-10:])
        left_fit_[2] = np.mean(self.left_c[-10:])
        right_fit_[0] = np.mean(self.right_a[-10:])
        right_fit_[1] = np.mean(self.right_b[-10:])
        right_fit_[2] = np.mean(self.right_c[-10:])
        # Generate x and y values for plotting
        ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
        left_fitx = left_fit_[0]*ploty**2 + left_fit_[1]*ploty + left_fit_[2]
        right_fitx = right_fit_[0]*ploty**2 + right_fit_[1]*ploty + right_fit_[2]

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 100]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 100, 255]

        return out_img, (left_fitx, right_fitx), (left_fit_, right_fit_), ploty

    def get_curve(self, img, leftx, rightx):
        # array from 0-479
        ploty = np.linspace(0, img.shape[0]-1, img.shape[0])

        # f = plt.figure()
        # ax1 = f.subplots()
        # ax1.plot(leftx, ploty)
        # plt.show()

        # get max
        y_eval = np.max(ploty)
        # ym_per_pix = 30.5/720  # meters per pixel in y dimension
        # xm_per_pix = 3.7/720  # meters per pixel in x dimension
        ym_per_pix = 100/480  # meters per pixel in y dimension
        xm_per_pix = 100/640  # meters per pixel in x dimension

        # Fit new polynomials to x,y in world space
       # print(ploty)
       # print(leftx)

        left_fit_cr = np.polyfit(ploty, leftx, 2)
        right_fit_cr = np.polyfit(ploty, rightx, 2)
        # print(left_fit_cr)

        left_fitx = left_fit_cr[0]*ploty**2 + left_fit_cr[1]*ploty + left_fit_cr[2]

        x1, y1 = 0, 0
        x2, y2 = 480, leftx[0] - leftx[-1]

        olen = x2 - x1
        alen = y2 - y1
        hlen = math.sqrt((olen**2)+(alen**2))
        theta = math.asin(olen/hlen)
        
        # Calculate the new radii of curvature
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])

        car_pos = img.shape[1]/2
        l_fit_x_int = left_fit_cr[0]*img.shape[0]**2 + left_fit_cr[1]*img.shape[0] + left_fit_cr[2]
        r_fit_x_int = right_fit_cr[0]*img.shape[0]**2 + right_fit_cr[1]*img.shape[0] + right_fit_cr[2]

        lane_center_position = (r_fit_x_int + l_fit_x_int) / 2
        center = (car_pos - lane_center_position)
        # print(l_fit_x_int, r_fit_x_int, car_pos - lane_center_position * xm_per_pix)
        # Now our radius of curvature is in meters
        return (left_curverad, right_curverad, center), theta

    def draw_lanes(self, img, left_fit, right_fit):
        ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
        color_img = np.zeros_like(img)
        left = np.array([np.transpose(np.vstack([left_fit, ploty]))])
        right = np.array([np.flipud(np.transpose(np.vstack([right_fit, ploty])))])
        points = np.hstack((left, right))

        cv2.fillPoly(color_img, np.int_(points), (0, 200, 255))
        inv_perspective = self.inv_perspective_warp(color_img)
        inv_perspective = cv2.addWeighted(img, 1, inv_perspective, 0.7, 0)
        return inv_perspective

    def detect_img(self, img_name):
        img = img_name
        # Gazebo image already in RGB
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_ = self.filter(img)
        # return img_
        # Gazebo return 640 x 640 image size
        img_ = self.perspective_warp(img_)
        # return img_

        out_img, curves, lanes, ploty = self.sliding_window(img_, margin=80)

        curverad, theta = self.get_curve(img, curves[0], curves[1])
        # return out_img


        lane_curve = np.mean([curverad[0], curverad[1]])
        img = self.draw_lanes(img, curves[0], curves[1])

        laneinfo = {"offset": "{:.4f}".format(curverad[2]),
                    "Theta": "{:.4f}".format(90-math.degrees(theta))}
        return img, str(laneinfo)

        # return 'Vehicle offset: {:.4f} cm\n'.format(curverad[2]) + 'Theta: {:.4f} degrees\n'.format(90-math.degrees(theta)) + '----\n'

