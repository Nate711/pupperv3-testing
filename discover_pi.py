import socket
import concurrent.futures

IP_PREFIX = "192.168.151."  # IP Prefix
PORT = 22  # SSH port
TIMEOUT = 0.5  # Socket connection timeout [s]


def check_ip(ip):
    try:
        # Create a socket to check if the IP is reachable
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(TIMEOUT)  # Set a short timeout for connection attempts
        s.connect((ip, PORT))  # You can change the port number if needed
        s.close()

        # If the connection succeeds, it means the IP is reachable
        print(f"IP Address found: {ip}")
        return ip
    except:
        return None


def discover_raspberry_pi_ip(network_prefix=IP_PREFIX):
    ips_to_check = [network_prefix + str(i) for i in range(1, 255)]

    with concurrent.futures.ThreadPoolExecutor() as executor:
        # Use the ThreadPoolExecutor to run the check_ip function in parallel
        results = executor.map(check_ip, ips_to_check)

    for ip in results:
        if ip:
            return ip

    print(f"No SSH'able devices found with the prefix: {network_prefix}")
    return None


if __name__ == "__main__":
    discover_raspberry_pi_ip()
