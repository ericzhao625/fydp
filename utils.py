import os
import psutil
import cv2

def monitor_memory():
    """
    Monitors and returns the current memory usage of the CV system.

    This function retrieves the resident set size (RSS) memory usage of the 
    current process using the `psutil` library.

    Returns:
        str: Formatted string representing the memory usage in MB.
    
    Dependencies:
        - `psutil` (install using `pip install psutil`)
        - `os` (part of Python standard library)
    """

    process = psutil.Process(os.getpid())

    memory_usage = process.memory_info().rss / (1024 * 1024)  # Memory in MB
    memory_usage = f"Memory Usage: {memory_usage:.2f} MB"
    return memory_usage


def display_metrics(frame, distance, pwm_value, pose_estimation):
    """
    Overlays system metrics onto the given frame.

    This function displays the estimated distance, PWM value, and pose estimation 
    results as text overlays on the provided OpenCV frame.

    Args:
        frame (np.ndarray): The image frame from OpenCV where text will be displayed.
        distance (Optional[float]): The estimated distance to the target in meters.
        pwm_value (Optional[float]): The PWM value controlling the motor, in percentage.
        pose_estimation (str): The pose estimation result (e.g., "centered", "move left", etc.).

    Returns:
        None: The function modifies the frame in place.

    Notes:
        - If both `distance` and `pwm_value` are provided, both are displayed.
        - If only `distance` is available, only distance is displayed.
        - Pose estimation is always displayed.
        - Text is drawn using OpenCV?s `cv2.putText()` function.

    Dependencies:
        - `cv2` (OpenCV library)
    """
    if distance and pwm_value:
        cv2.putText(
            frame, f'Estimated Distance: {distance:.2f}m, PWM: {pwm_value:.2f}%',
            (0, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
        )
    elif distance:
        cv2.putText(
            frame, f'Estimated Distance: {distance:.2f}m',
            (0, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
        )

    cv2.putText(
        frame, f'Pose Estimation: {pose_estimation}',
        (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
    )
