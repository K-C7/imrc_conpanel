import subprocess
import glob
import re


def get_tty_by_serial(serial):
    for dev in get_tty_device_list():
        if(dev['serial'] == serial):
            return dev['path']
    
    return None

def get_tty_device_list():
    # /dev/tty* に一致するデバイスパスをすべて取得
    devices = sorted(glob.glob('/dev/tty*'))
    
    if not devices:
        return

    device_list = []

    for dev_path in devices:
        try:
            # udevadm info コマンドを実行
            result = subprocess.run(
                ['/bin/udevadm', 'info', f'--name={dev_path}'],
                capture_output=True,
                text=True,
                check=True
            )
            
            # ID_SERIAL_SHORT の行を探す
            serial = "Unknown"
            for line in result.stdout.splitlines():
                if "ID_SERIAL_SHORT=" in line:
                    serial = line.split('=')[1]
                    break
            
            if(serial != "Unknown"):
                # print(f"{dev_path:<15} {serial}")
                device_list.append({'path': dev_path, 'serial': serial})

        except subprocess.CalledProcessError:
            pass
    return device_list

def display_device_list():
    # /dev/tty* に一致するデバイスパスをすべて取得
    devices = sorted(glob.glob('/dev/tty*'))
    
    if not devices:
        print("No /dev/ttyACM devices found.")
        return

    print(f"{'Device Path':<15} {'Serial Number'}")
    print("-" * 45)

    device_list = []

    for dev_path in devices:
        try:
            # udevadm info コマンドを実行
            result = subprocess.run(
                ['/bin/udevadm', 'info', f'--name={dev_path}'],
                capture_output=True,
                text=True,
                check=True
            )
            
            # ID_SERIAL_SHORT の行を探す
            serial = "Unknown"
            for line in result.stdout.splitlines():
                if "ID_SERIAL_SHORT=" in line:
                    serial = line.split('=')[1]
                    break
            
            if(serial != "Unknown"):
                print(f"{dev_path:<15} {serial}")
                device_list.append({'path': dev_path, 'serial': serial})

        except subprocess.CalledProcessError:
            print(f"{dev_path:<15} Error: Could not retrieve info")

if __name__ == "__main__":
    # 実行して結果をリスト形式で受け取る
    acm_devices = get_tty_device_list()