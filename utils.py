import psutil
import cv2

def monitor_memory():
    """
    Monitor memory usage from CV system

    :return: memory usage
    """

    process = psutil.Process(os.getpid())

    memory_usage = process.memory_info().rss / (1024 * 1024)  # Memory in MB
    memory_usage = f"Memory Usage: {memory_usage:.2f} MB"
    return memory_usage


def display_metrics(frame, distance, pwm_value, pose_estimation):
    if distance and pwm_value:
        cv2.putText(frame, f'Estimated Distance: {distance:.2f}m, PWM: {pwm_value:.2f}%', (0, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    elif distance:
        cv2.putText(frame, f'Estimated Distance: {distance:.2f}m', (0, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.putText(frame, f'Pose Estimation: {pose_estimation}', (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
