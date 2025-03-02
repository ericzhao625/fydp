import psutil


def monitor_memory():
    """
    Monitor memory usage from CV system

    :return: memory usage
    """

    process = psutil.Process(os.getpid())

    memory_usage = process.memory_info().rss / (1024 * 1024)  # Memory in MB
    memory_usage = f"Memory Usage: {memory_usage:.2f} MB"
    return memory_usage