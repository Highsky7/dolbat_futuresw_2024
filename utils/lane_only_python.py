import cv2
import numpy as np
import matplotlib.pyplot as plt


# 전역 변수로 이전 프레임의 차선 위치를 저장
prev_left_base = None
prev_right_base = None

# Fixed Bird Eye's View Points
fixed_points = [(6, 472), (590, 472), (396, 250), (147, 250)]

# 픽셀 당 거리(m/pixel)
x_m_per_pixel = 1.62367454495 * 10 ** -3  # 픽셀 당 거리 (1.62367454495mm -> 1.62367454495 * 10 ** -3 m/pixel)
y_m_per_pixel = 3.643428499552204 * 10 ** -3  # 픽셀 당 거리 (3.643428499552204mm -> 3.643428499552204 * 10 ** -3 m/pixel)
real_shift_distance = 0.45  # 실제 이동해야 할 거리 (0.45m, 즉 450mm)
pixel_shift = real_shift_distance / x_m_per_pixel  # 실제 평행이동할 픽셀 거리


L_m = 55 * 10 ** -2


def get_bird_eye_view(image, output_size, points):
    if len(points) != 4:
        return image

    height, width = output_size[1], output_size[0]
    src_points = np.float32([points[0], points[1], points[3], points[2]])
    dst_points = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    bird_eye_view = cv2.warpPerspective(image, matrix, output_size)

    return bird_eye_view


def mask_rgb_areas(image, lower_rgb, upper_rgb, blur_ksize=(15, 15)):
    blurred_image = cv2.GaussianBlur(image, blur_ksize, 0)
    mask = cv2.inRange(blurred_image, np.array(lower_rgb), np.array(upper_rgb))
    return mask


def sliding_window_demo(image, nwindows=18, margin=50, minpix=1):
    global prev_left_base, prev_right_base

    lower_rgb = [225, 225, 225]
    upper_rgb = [255, 255, 255]

    masked_image = mask_rgb_areas(image, lower_rgb, upper_rgb)
    bottom_third = masked_image.shape[0] * 2 // 3
    histogram = np.sum(masked_image[bottom_third:, :], axis=0)

    midpoint = int(histogram.shape[0] // 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    left_lane_boundary = image.shape[1] // 3
    right_lane_boundary = 1.6 * image.shape[1] // 3

    apply_left_window = left_lane_boundary >= leftx_base > 0
    apply_right_window = rightx_base >= right_lane_boundary and rightx_base > midpoint

    if not apply_left_window and prev_left_base is not None:
        leftx_base = prev_left_base
    else:
        prev_leftx_base = leftx_base

    if not apply_right_window and prev_right_base is not None:
        rightx_base = prev_right_base
    else:
        prev_rightx_base = rightx_base

    window_height = int(masked_image.shape[0] // nwindows)
    leftx_current = leftx_base
    rightx_current = rightx_base

    nonzero = masked_image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    left_lane_inds = []
    right_lane_inds = []

    left_windows = []
    right_windows = []

    left_lane_points_x = []
    left_lane_points_y = []
    right_lane_points_x = []
    right_lane_points_y = []

    for window in range(nwindows):
        win_y_low = masked_image.shape[0] - (window + 1) * window_height
        win_y_high = masked_image.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        if apply_left_window:
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            if len(good_left_inds) > minpix:
                left_lane_inds.append(good_left_inds)
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
                left_windows.append((win_xleft_low, win_y_low, win_xleft_high, win_y_high))

                midpoint_x = int(np.mean(nonzerox[good_left_inds]))
                midpoint_y = int(np.mean(nonzeroy[good_left_inds]))
                left_lane_points_x.append(midpoint_x)
                left_lane_points_y.append(midpoint_y)

        if apply_right_window:
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            if len(good_right_inds) > minpix:
                right_lane_inds.append(good_right_inds)
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
                right_windows.append((win_xright_low, win_y_low, win_xright_high, win_y_high))

                midpoint_x = int(np.mean(nonzerox[good_right_inds]))
                midpoint_y = int(np.mean(nonzeroy[good_right_inds]))
                right_lane_points_x.append(midpoint_x)
                right_lane_points_y.append(midpoint_y)

    if len(left_lane_points_x) >= 4 and len(left_lane_points_y) >= 4:
        left_fit = np.polyfit(left_lane_points_y, left_lane_points_x, 3)
    else:
        left_fit = None

    if len(right_lane_points_x) >= 4 and len(right_lane_points_y) >= 4:
        right_fit = np.polyfit(right_lane_points_y, right_lane_points_x, 3)
    else:
        right_fit = None

    return left_fit, right_fit


def calculate_path(left_fit, right_fit, image_shape, pixel_shift):
    ploty = np.linspace(0, image_shape[0] - 1, image_shape[0])

    if left_fit is not None and right_fit is not None:
        left_fitx = left_fit[0] * ploty ** 3 + left_fit[1] * ploty ** 2 + left_fit[2] * ploty + left_fit[3]
        right_fitx = right_fit[0] * ploty ** 3 + right_fit[1] * ploty ** 2 + right_fit[2] * ploty + right_fit[3]
        center_fitx = (left_fitx + right_fitx) / 2
        return ploty, center_fitx, left_fitx, right_fitx

    elif right_fit is not None:
        right_fitx = right_fit[0] * ploty ** 3 + right_fit[1] * ploty ** 2 + right_fit[2] * ploty + right_fit[3]
        center_fitx = right_fitx - pixel_shift
        return ploty, center_fitx, None, right_fitx

    elif left_fit is not None:
        left_fitx = left_fit[0] * ploty ** 3 + left_fit[1] * ploty ** 2 + left_fit[2] * ploty + left_fit[3]
        center_fitx = left_fitx + pixel_shift
        return ploty, center_fitx, left_fitx, None

    else:
        return None, None, None, None


def calculate_steering_angle(x_la, y_la):
    # 조향각을 arctan을 사용하여 계산 (라디안 값)
    steering_angle = np.arctan2(2 * L_m * y_la, x_la ** 2 + y_la ** 2)
    return np.degrees(steering_angle)  # 라디안을 도로 변환


def process_video_feed(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Cannot open the video.")
        return

    plt.ion()  # 인터랙티브 모드 켜기
    fig, ax = plt.subplots()

    # 축의 범위 설정
    ax.set_xlim(-0.485, 0.552)  # x축 범위 고정
    ax.set_ylim(0.00364, 1.748)  # y축 범위 고정


def process_video_feed(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Cannot open the video.")
        return

    plt.ion()  # Interactive mode on
    fig, ax = plt.subplots()

    # Set axis limits
    ax.set_xlim(0, 2)  # x-axis fixed range
    ax.set_ylim(-0.5, 0.5)  # y-axis fixed range

    x_la = 1.5  # Lookahead X value in meters

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        bev = get_bird_eye_view(frame, (frame.shape[1], frame.shape[0]), fixed_points)
        left_fit, right_fit = sliding_window_demo(bev)
        ploty, center_fitx, left_fitx, right_fitx = calculate_path(left_fit, right_fit, bev.shape, pixel_shift)

        if ploty is not None and center_fitx is not None:
            ax.clear()

            # Convert coordinates to meters
            ploty_m = (480 - ploty) * y_m_per_pixel

            if left_fitx is not None:
                left_fitx_m = (340 - left_fitx) * x_m_per_pixel
                ax.plot(ploty_m, left_fitx_m, color='blue', label='Left Lane')

            if right_fitx is not None:
                right_fitx_m = (340 - right_fitx) * x_m_per_pixel
                ax.plot(ploty_m, right_fitx_m, color='red', label='Right Lane')

            center_fitx_m = (340 - center_fitx) * x_m_per_pixel
            ax.plot(ploty_m, center_fitx_m, color='orange', label='Desired Path')

            # Calculate y_la corresponding to x_la
            center_fit_coeffs = np.polyfit(ploty_m, center_fitx_m, 3)
            y_la = np.polyval(center_fit_coeffs, x_la)

            # Plot the lookahead point
            ax.plot(x_la, y_la, 'go', markersize=10, label='Look Ahead Point')

            # Calculate steering angle
            steering_angle = calculate_steering_angle(x_la, y_la)
            print(f"Steering Angle: {steering_angle:.2f} degrees")

            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Y Position (m)")
            ax.set_ylim(-0.5, 0.5)  # Fix y-axis range
            ax.set_xlim(0, 2)  # Fix x-axis range
            ax.legend(loc='lower right')
            plt.draw()
            plt.pause(0.001)

        cv2.imshow("Video", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    plt.ioff()  # Interactive mode off
    plt.show()


if __name__ == "__main__":
    video_path = "trackrecord4.mp4"
    process_video_feed(video_path)
