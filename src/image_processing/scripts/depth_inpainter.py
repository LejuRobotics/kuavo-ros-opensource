#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import time
import rospy
from collections import deque
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError


class DepthImageInpainter:
    def __init__(self):
        """初始化节点和参数"""
        rospy.init_node('depth_image_inpainter', anonymous=True)
        
        self.get_parameters()
        self.bridge = CvBridge()
        self.methods = {
            'telea': cv2.INPAINT_TELEA,
            'ns': cv2.INPAINT_NS
        }
        # self.history = deque(maxlen=3)
        # 深度历史环形缓冲区
        self.buf_size = 43
        self.depth_buf = np.zeros((self.buf_size, 36, 64), dtype=np.float64)
        self.cur_buf_idx = 0
        self.selected_ids = [0, 6, 12, 18, 24, 30, 36, 42]
        self.depth_buf_filled = False
        self.last_delay_print_time = 0.0
        self.setup_publishers_subscribers()
    
    def get_parameters(self):
        """从参数服务器获取参数"""
        self.input_topic = rospy.get_param('~input_topic', '/camera/depth/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/camera/depth/image_processed')
        
        self.queue_size = rospy.get_param('~queue_size', 1)
        self.buff_size = rospy.get_param('~buff_size', 2**24)  # 16MB

        self.use_white_spot_removal = rospy.get_param('~use_white_spot_removal', True)
        self.fst_spot_ddiff = rospy.get_param('~fst_spot_ddiff', 0.5)   # m
        self.fst_wh_thres = rospy.get_param('~fst_wh_thres', 3.0)
        self.fst_max_spot_size = rospy.get_param('~fst_max_spot_size', 512)   # pixels
        self.fst_sur_black_ratio = rospy.get_param('~fst_sur_black_ratio', 0.5)
        self.sec_spot_ddiff = rospy.get_param('~sec_spot_ddiff', 0.5)   # m
        self.sec_wh_thres = rospy.get_param('~sec_wh_thres', 1.0)
        self.sec_max_spot_size = rospy.get_param('~sec_max_spot_size', 512)   # pixels
        self.sec_sur_black_ratio = rospy.get_param('~sec_sur_black_ratio', 0.5)

        self.min_depth = rospy.get_param('~min_depth', 0.0)
        self.min_depth_physics = rospy.get_param('~min_depth_physics', 0.17)
        self.max_depth = rospy.get_param('~max_depth', 2.5)

        self.resize_scale = rospy.get_param('~resize_scale', 0.5)

        self.crop_top = rospy.get_param('~crop_top', 0)
        self.crop_bottom = rospy.get_param('~crop_bottom', 0)
        self.crop_left = rospy.get_param('~crop_left', 0)
        self.crop_right = rospy.get_param('~crop_right', 0)

        self.inpaint_radius = rospy.get_param('~inpaint_radius', 3)
        self.inpaint_method = rospy.get_param('~inpaint_method', 'telea')
        
        self.use_gaussian = rospy.get_param('~use_gaussian', False)
        self.gaussian_kernel_size = rospy.get_param('~gaussian_kernel_size', 3)
        self.gaussian_sigma_x = rospy.get_param('~gaussian_sigma_x', 1.0)
        self.gaussian_sigma_y = rospy.get_param('~gaussian_sigma_y', 1.0)

        self.output_encoding = rospy.get_param('~output_encoding', 'same_as_input')

        self.debug = rospy.get_param('~debug', False)
        self.runtime = rospy.get_param('~runtime', False)

        self.show_histogram = rospy.get_param('~show_histogram', False)
        self.histogram_bins = rospy.get_param('~histogram_bins', 125)
        self.histogram_width = rospy.get_param('~histogram_width', 800)
        self.histogram_height = rospy.get_param('~histogram_height', 450)
        self.histogram_min_depth = rospy.get_param('~histogram_min_depth', 0.18)
        self.histogram_max_depth = rospy.get_param('~histogram_max_depth', 2.5)
    
    def setup_publishers_subscribers(self):
        self.image_sub = rospy.Subscriber(
            self.input_topic,
            Image,
            self.image_callback,
            queue_size=self.queue_size,
            buff_size=self.buff_size
        )

        if self.debug:
            self.histogram_pub = rospy.Publisher('/camera/debug/process/0_raw_depth_histogram', Image, queue_size=self.queue_size)

            self.white_spot_mask_pub = rospy.Publisher('/camera/debug/process/1_white_spot/mask', Image, queue_size=self.queue_size)
            self.white_spot_removal_pub = rospy.Publisher('/camera/debug/process/1_white_spot/removal', Image, queue_size=self.queue_size)
            self.white_spot_exclusion_pub = rospy.Publisher('/camera/debug/process/1_white_spot/exclusion', Image, queue_size=self.queue_size)
            self.removal_pub = rospy.Publisher('/camera/debug/process/1_white_spot/post_removal_depth', Image, queue_size=self.queue_size)

            self.mask_pub = rospy.Publisher('/camera/debug/process/2_mask', Image, queue_size=self.queue_size)
            self.cropped_pub = rospy.Publisher('/camera/debug/process/3_cropped_depth', Image, queue_size=self.queue_size)
            self.resized_pub = rospy.Publisher('/camera/debug/process/4_resized_depth', Image, queue_size=self.queue_size)
            self.inpainted_pub = rospy.Publisher('/camera/debug/process/5_inpainted_depth', Image, queue_size=self.queue_size)
            
            self.raw_resize_pub = rospy.Publisher('/camera/debug/diff/raw_resize', Image, queue_size=self.queue_size)
            self.pos_diff_pub = rospy.Publisher('/camera/debug/diff/pos_diff', Image, queue_size=self.queue_size)
            self.neg_diff_pub = rospy.Publisher('/camera/debug/diff/neg_diff', Image, queue_size=self.queue_size)
        
        self.blurred_pub = rospy.Publisher('/camera/depth/image_blurred', Image, queue_size=self.queue_size)
        self.nomalized_pub = rospy.Publisher(self.output_topic, Image, queue_size=self.queue_size)
        self.depth_history_array_pub = rospy.Publisher('/camera/depth/depth_history_array', Float64MultiArray, queue_size=self.queue_size)

    def create_mask(self, depth_image):
        """检测深度缺失区域"""
        mask = np.zeros(depth_image.shape[:2], dtype=np.uint8)
        # mask[depth_image < depth_image.min() + 0.2] = 255
        mask[depth_image == 0] = 255
        
        # 扩大修复区域
        if np.sum(mask) > 0:
            kernel = np.ones((2, 2), np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=1)
        return mask
    
    def image_callback(self, msg):
        try:
            publish_time =msg.header.stamp.to_sec()
            current_time = rospy.Time.now().to_sec()
            if current_time - self.last_delay_print_time >= 1.0:
                print("delay time", current_time - publish_time)
                self.last_delay_print_time = current_time
            if self.runtime:
                start_time = time.time()
            if self.debug and not self.runtime:
                start_time = time.time()
            
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            except CvBridgeError as e:
                if self.debug:
                    rospy.logdebug(f"Encoding failure: {e}")
            
            # encoding / unit convertion
            cv_image = cv_image.astype(np.float32) / 1000.0  # mm -> m
            orig_h, orig_w = cv_image.shape[:2]
            new_h = int(orig_h * 0.5)
            new_w = int(orig_w * 0.5)
            cv_image = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
            if self.debug:
                rospy.loginfo(
                    f"""Enc={msg.encoding}, H={msg.height}, W={msg.width} dtype={cv_image.dtype}, S={cv_image.shape}, min={np.nanmin(cv_image):.1f}, max={np.nanmax(cv_image):.1f}"""
                )
        
            # cv_image = np.clip(cv_image, self.min_depth, self.max_depth)

            # real-time histogram
            if self.debug:
                hist_image = self.create_histogram_image(
                    cv_image, 
                    hist_min=self.histogram_min_depth, 
                    hist_max=self.histogram_max_depth,
                    bins=self.histogram_bins, 
                    width=self.histogram_width, 
                    height=self.histogram_height
                )
                hist_msg = self.bridge.cv2_to_imgmsg(hist_image, encoding='bgr8')
                hist_msg.header.stamp = rospy.Time.now()
                self.histogram_pub.publish(hist_msg)
                if self.show_histogram:
                    cv2.imshow('Raw Depth Histogram', hist_image)
                    cv2.waitKey(1)
                    
                raw_cropped = self.apply_crop(cv_image)
                raw_resized = self.apply_resize(raw_cropped, interpolation=cv2.INTER_NEAREST)
                msg = self.bridge.cv2_to_imgmsg(raw_resized, encoding='32FC1')
                msg.header.stamp = rospy.Time.now()
                self.raw_resize_pub.publish(msg)

            cv_image = self.remove_white_spots(
                cv_image, 
                depth_diff=self.fst_spot_ddiff, 
                white_thresh=self.fst_wh_thres, 
                max_spot_size=self.fst_max_spot_size, 
                surround_black_ratio=self.fst_sur_black_ratio
            )
            cv_image = self.remove_white_spots(
                cv_image, 
                depth_diff=self.sec_spot_ddiff, 
                white_thresh=self.sec_wh_thres, 
                max_spot_size=self.sec_max_spot_size, 
                surround_black_ratio=self.sec_sur_black_ratio
            )
            if self.debug:
                removal_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='32FC1')
                removal_msg.header.stamp = rospy.Time.now()
                self.removal_pub.publish(removal_msg)

            if self.debug:
                mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
                mask[cv_image == 0] = 255
                # mask[cv_image < 0.2] = 255
                # mask[cv_image == 0] = 0
                cv2.imshow('Depth mask', mask)
                cv2.waitKey(1)

            # create mask for repair
            mask_time = time.time() 

            mask = self.create_mask(cv_image)
            if self.debug:
               

                mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
                mask_msg.header = msg.header
                self.mask_pub.publish(mask_msg)

            img_cropped = self.apply_crop(cv_image)
            mask = self.apply_crop(mask)
            if self.debug:
                cropped_msg = self.bridge.cv2_to_imgmsg(img_cropped, encoding='32FC1')
                cropped_msg.header.stamp = rospy.Time.now()
                self.cropped_pub.publish(cropped_msg)

            img_resized = self.apply_resize(img_cropped, interpolation=cv2.INTER_NEAREST)
            mask = self.apply_resize(mask, interpolation=cv2.INTER_NEAREST)
            if self.debug:
                resized_msg = self.bridge.cv2_to_imgmsg(img_resized, encoding='32FC1')
                resized_msg.header.stamp = rospy.Time.now()
                self.resized_pub.publish(resized_msg)

            # img_resized = np.clip(img_resized, self.min_depth, self.max_depth)
            
            img_inpainted = self.inpaint_depth_image(img_resized, mask)
            inpaint_time = time.time() 
            if self.debug:
                
                # print(f"inpainted image min: {img_inpainted.min()}")
                missing_pixels = np.sum(mask > 0)
                total_pixels = mask.shape[0] * mask.shape[1]
                missing_ratio = missing_pixels / total_pixels * 100
                rospy.loginfo(f"Mask: {missing_pixels} pixels ({missing_ratio:.2f}%)")

            if self.debug:
                if self.show_histogram:
                    hist_image = self.create_histogram_image(
                            img_inpainted, 
                            hist_min=0.0, 
                            hist_max=self.histogram_max_depth,
                            bins=self.histogram_bins, 
                            width=self.histogram_width, 
                            height=self.histogram_height
                        )
                    cv2.imshow('Inpainted Depth Histogram', hist_image)
                inpainted_msg = self.bridge.cv2_to_imgmsg(img_inpainted, encoding='32FC1')
                inpainted_msg.header = msg.header
                self.inpainted_pub.publish(inpainted_msg)

                pos_diff = (img_inpainted - raw_resized).clip(min=0)
                neg_diff = (raw_resized - img_inpainted).clip(min=0)
                msg = self.bridge.cv2_to_imgmsg(pos_diff, encoding='32FC1')
                msg.header.stamp = rospy.Time.now()
                self.pos_diff_pub.publish(msg)
                msg = self.bridge.cv2_to_imgmsg(neg_diff, encoding='32FC1')
                msg.header.stamp = rospy.Time.now()
                self.neg_diff_pub.publish(msg)

            img_blurred = self.apply_gaussian_blur(img_inpainted)
            blur_time = time.time() 
                
            
            img_normalized = np.clip(img_blurred, 0, self.max_depth) / self.max_depth
            processed_msg = self.bridge.cv2_to_imgmsg(img_normalized, encoding='32FC1')
            processed_msg.header = msg.header
            self.nomalized_pub.publish(processed_msg)

            # if self.output_encoding == 'same_as_input':
                # output_encoding = msg.encoding
            # else:
                # output_encoding = self.output_encoding
            # output_image = self.prepare_output_image(img_inpainted, output_encoding)
            
            if self.cur_buf_idx == 0 and not self.depth_buf_filled:
                self.depth_buf[:] = np.repeat(img_normalized[None, :, :], self.buf_size, axis=0)
            else:
                self.depth_buf[:] = np.concatenate((self.depth_buf[1:], img_normalized[None, :, :]), axis=0)
            if self.cur_buf_idx >= (self.buf_size - 1) and not self.depth_buf_filled:
                self.depth_buf_filled = True
            self.cur_buf_idx = (self.cur_buf_idx + 1) % self.buf_size
            depth_history_stack = self.depth_buf[self.selected_ids].reshape(-1).astype(np.float64)
            history_msg = Float64MultiArray()
            history_msg.data = depth_history_stack.tolist()
            self.depth_history_array_pub.publish(history_msg)
            
            # if self.debug:
                # rospy.loginfo(f"Processed image: Enc={output_encoding}, dtype={output_image.dtype}, S={output_image.shape}, min={np.nanmin(output_image):.1f}, max={np.nanmax(output_image):.1f}")
            if self.runtime and not self.debug:
                rospy.loginfo(f"Runtime >> Mask: {(mask_time-start_time)*1000:.2f} ms | Inpainting: {(inpaint_time-mask_time)*1000:.2f} ms | GaussianBlur: {(blur_time-inpaint_time)*1000:.2f} ms | Total: {(time.time() - start_time)*1000:.2f} ms")

            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Image Processing Error: {e}")
            rospy.logerr(f"Error details: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def prepare_output_image(self, depth_image, output_encoding):
        if output_encoding == '32FC1':
            return depth_image.astype(np.float32)
        elif output_encoding == '16UC1':
            # 将米转换为毫米，并限制在16位范围内
            depth_mm = depth_image * 1000.0
            depth_mm = np.clip(depth_mm, 0, 65535)  # 16位最大值
            return depth_mm.astype(np.uint16)
        else:
            rospy.logwarn(f"Unsupported Encoding: {output_encoding}, use 32FC1 instead")
            return depth_image.astype(np.float32)
    
    def remove_white_spots(
            self, depth, depth_diff=0.5, white_thresh=5.0, max_spot_size=80, surround_black_ratio=0.3
        ):
        """
        Remove white spot noise (depth > white_thresh) that:
        - size < max_spot_size
        - surrounded by black region (depth < component_depth - depth_diff)
        """
        if not self.use_white_spot_removal:
            return depth
        depth_out = depth.copy()

        white_mask = depth > white_thresh

        if self.debug:
            msg = self.bridge.cv2_to_imgmsg((255*white_mask).astype(np.uint8), encoding='mono8')
            msg.header.stamp = rospy.Time.now()
            self.white_spot_mask_pub.publish(msg)

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            white_mask.astype(np.uint8),
            connectivity=8
        )

        if num_labels <= 1:  # only background
            return depth_out

        # areas for each component (excluding background)
        areas = stats[1:, cv2.CC_STAT_AREA]

        # compute mean depth per component using vectorized bincount
        white_pixel_indices = np.where(white_mask)
        white_pixel_labels = labels[white_pixel_indices]  # labels from 1..num_labels-1
        white_pixel_depths = depth[white_pixel_indices]
        # bincount requires labels starting from 0, but white_pixel_labels >=1
        depth_sums = np.bincount(white_pixel_labels, white_pixel_depths, minlength=num_labels)
        pixel_counts = np.bincount(white_pixel_labels, minlength=num_labels)
        # avoid division by zero (should not happen for labels present in white_mask)
        component_depth_means = depth_sums / (pixel_counts + 1e-7)

        # dilate the label image to obtain border pixels
        kernel = np.ones((3, 3), np.uint8)
        # use uint16 to avoid overflow of label values
        dilated_labels = cv2.dilate(labels.astype(np.uint16), kernel)
        border_mask = (dilated_labels > 0) & (labels == 0)
        
        if not np.any(border_mask):
            # no border pixels (unlikely)
            return depth_out
        
        border_labels = dilated_labels[border_mask]  # labels of adjacent components
        border_depths = depth[border_mask]
        
        # thresholds per component
        thresholds = component_depth_means - depth_diff
        # compare each border pixel depth with its corresponding component threshold
        border_is_black = border_depths < thresholds[border_labels]
        
        # accumulate black counts and total border counts per component
        black_counts = np.bincount(border_labels, weights=border_is_black.astype(int), minlength=num_labels)
        border_counts = np.bincount(border_labels, minlength=num_labels)
        
        # compute black ratio, avoid division by zero (set ratio to 0 where no border)
        black_ratio = np.zeros(num_labels, dtype=np.float32)
        non_zero = border_counts > 0
        black_ratio[non_zero] = black_counts[non_zero] / border_counts[non_zero]
        
        # decide which components to remove
        # (skip background label 0)
        candidate_mask = (areas < max_spot_size) & (black_ratio[1:] >= surround_black_ratio)
        remove_labels = np.where(candidate_mask)[0] + 1  # convert to original label indices
        
        if len(remove_labels) > 0:
            # create removal mask using np.isin (vectorized)
            remove_mask = np.isin(labels, remove_labels)
            depth_out[remove_mask] = 0.0

            if self.debug:
                remove_img = (remove_mask * 255).astype(np.uint8)
                msg = self.bridge.cv2_to_imgmsg(remove_img, encoding='mono8')
                msg.header.stamp = rospy.Time.now()
                self.white_spot_removal_pub.publish(msg)

                remaining_mask = (white_mask & (~remove_mask)).astype(np.uint8) * 255
                msg = self.bridge.cv2_to_imgmsg(remaining_mask, encoding='mono8')
                msg.header.stamp = rospy.Time.now()
                self.white_spot_exclusion_pub.publish(msg)

        return depth_out

    def inpaint_depth_image(self, image, mask):
        if image.dtype != np.float32:
            depth_image_float = image.astype(np.float32)
        else:
            depth_image_float = image

        valid_mask = np.isfinite(depth_image_float) & (depth_image_float > 0)

        min_val_phy = depth_image_float[valid_mask].min()
        max_val = depth_image_float.max()

        # normalize depth to 0-255
        gray_scale = np.zeros_like(depth_image_float, dtype=np.float32)
        gray_scale[valid_mask] = (depth_image_float[valid_mask] - min_val_phy) / (max_val - min_val_phy) * 255.0
        gray_scale = np.clip(gray_scale, 0, 255).astype(np.uint8)

        # apply inpainting
        inpainted_gray_scale = cv2.inpaint(
            gray_scale,
            mask,
            5,
            cv2.INPAINT_TELEA
        )

        inpainted_gray_scale = cv2.inpaint(
            gray_scale,
            mask,
            self.inpaint_radius,
            self.methods.get(self.inpaint_method, cv2.INPAINT_NS)
        )
        
        # convertion and normalization
        inpainted_depth = inpainted_gray_scale.astype(np.float32) / 255.0
        inpainted_depth = (inpainted_depth * (max_val - min_val_phy) + min_val_phy)
        inpainted_depth[valid_mask] = depth_image_float[valid_mask]
        return inpainted_depth
    
    def apply_resize(self, image, interpolation=cv2.INTER_NEAREST):
        if self.resize_scale != 1.0 and self.resize_scale > 0:
            orig_h, orig_w = image.shape[:2]
            new_h = int(orig_h * self.resize_scale*2)
            new_w = int(orig_w * self.resize_scale*2)
            image = cv2.resize(image, (new_w, new_h), interpolation=interpolation)
            if self.debug:
                rospy.loginfo(f"Resized image to scale {self.resize_scale} ({new_h}x{new_w})")
        return image

    def apply_crop(self, image):
        h, w = image.shape[:2]
        crop_h_start = self.crop_top
        crop_h_end = h - self.crop_bottom
        crop_w_start = self.crop_left
        crop_w_end = w - self.crop_right
        if crop_h_start < crop_h_end and crop_w_start < crop_w_end:
            image = image[crop_h_start:crop_h_end, crop_w_start:crop_w_end]
            if self.debug:
                rospy.loginfo(f"Cropped image to ({crop_h_start}:{crop_h_end}, {crop_w_start}:{crop_w_end})")
        return image
    
    def apply_gaussian_blur(self, image):
        kernel_size = self.gaussian_kernel_size
        if kernel_size % 2 == 0:
            kernel_size += 1
            rospy.logwarn_once(f"The Gaussian kernel size must be odd. Adjust it to {kernel_size}")

        blurred = cv2.GaussianBlur(
            image, 
            (kernel_size, kernel_size), 
            sigmaX=self.gaussian_sigma_x,
            sigmaY=self.gaussian_sigma_y
        )
        blurred_msg = self.bridge.cv2_to_imgmsg(blurred, encoding='32FC1')
        blurred_msg.header.stamp = rospy.Time.now()
        self.blurred_pub.publish(blurred_msg)
        
        return blurred
    
    def create_histogram_image(self, depth_image, hist_min=0, hist_max=1.0, bins=10, width=400, height=300):
        """
        根据深度图像创建直方图可视化图像
        深度图像应为浮点型, (单位：米), 无效深度值为 0
        返回一个 RGB 图像 (numpy数组) 用于显示直方图
        """
        assert hist_min < hist_max, "Histogram min depth must be less than max depth"

        # 提取有效深度值（>0）
        valid_depths = depth_image[depth_image > 0].flatten()
        num_invalid = len(depth_image[depth_image==0].flatten())
        invalid_ratio = num_invalid/len(depth_image.flatten())
        rospy.loginfo(f"Invalid depth: {num_invalid}, ratio: {invalid_ratio*100:.1f}%")
        if len(valid_depths) == 0:
            hist_image = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(hist_image, "No valid depths", (10, height//2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            return hist_image

        hist, bin_edges = np.histogram(depth_image, bins=bins, range=(hist_min, hist_max))
        max_count = np.max(hist) if np.max(hist) > 0 else 1
        hist_normalized = (hist / max_count) * (height - 20)  # 保留边距
        
        hist_image = np.zeros((height, width, 3), dtype=np.uint8)
        bin_width = width // bins
        
        for i in range(bins):
            x1 = i * bin_width
            x2 = (i + 1) * bin_width
            y1 = height - int(hist_normalized[i])
            y2 = height
            if hist[i] > 0:
                color = (0, 255, 0)
                cv2.rectangle(hist_image, (x1, y1), (x2, y2), color, -1)
                cv2.rectangle(hist_image, (x1, y1), (x2, y2), (255, 255, 255), 1)

        cv2.line(hist_image, (0, height-1), (width, height-1), (255, 255, 255), 2)  # x轴
        cv2.line(hist_image, (0, 0), (0, height), (255, 255, 255), 2)  # y轴
 
        cv2.putText(hist_image, f"Depth Histogram (min={hist_min:.2f}m, max={hist_max:.2f}m)",
                   (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(hist_image, f"Total valid pixels: {len(valid_depths)} ({(1-invalid_ratio)*100:.1f}%) ",
                   (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(hist_image, f"Bin count: {bins}",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # 显示当前帧的深度范围
        depth_min = np.min(valid_depths)
        depth_max = np.max(valid_depths)
        cv2.putText(hist_image, f"Curr depth range: [{depth_min:.2f}, {depth_max:.2f}] m",
                   (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        return hist_image
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DepthImageInpainter()
        node.run()
    except rospy.ROSInterruptException:
        pass
