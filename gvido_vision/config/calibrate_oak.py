#!/usr/bin/env python3
import depthai as dai
import json

print("=" * 60)
print("ПРОВЕРКА КАЛИБРОВКИ OAK-D-LITE")
print("=" * 60)

# Подключаемся к устройству
device = dai.Device(dai.UsbSpeed.HIGH)

print(f"📷 MXID: {device.getDeviceId()}")
print(f"📷 USB скорость: {device.getUsbSpeed()}")

# Читаем калибровку
calib_data = device.readCalibration()
eeprom = calib_data.getEepromData()

print(f"\n📊 EEPROM версия: {eeprom.version}")
print(f"📊 Board: {eeprom.boardName}")

if eeprom.version == 0:
    print("\n⚠️ КАЛИБРОВКА ОТСУТСТВУЕТ!")
else:
    print("\n✅ КАЛИБРОВКА ПРИСУТСТВУЕТ!")
    
    # Получаем матрицы для левой камеры
    try:
        intrinsics = calib_data.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 640, 480)
        print(f"\n🎯 Матрица левой камеры (intrinsics):")
        print(f"   fx = {intrinsics[0][0]:.2f}, fy = {intrinsics[1][1]:.2f}")
        print(f"   cx = {intrinsics[0][2]:.2f}, cy = {intrinsics[1][2]:.2f}")
    except Exception as e:
        print(f"\n⚠️ Не удалось получить матрицы: {e}")
    
    # Получаем экстринсики (расстояние между камерами)
    try:
        extrinsics = calib_data.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)
        baseline = extrinsics[0][3]  # расстояние по X
        print(f"\n🔧 Расстояние между камерами (baseline): {baseline*100:.1f} см")
        
        if baseline < 0:
            print("   ⚠️ Отрицательное значение! Камеры перепутаны местами.")
        elif baseline > 0.1:
            print("   ⚠️ Слишком большое расстояние! (>10 см)")
        elif baseline > 0.07 and baseline < 0.08:
            print("   ✅ Нормальное расстояние для OAK-D-LITE (~7.5 см)")
    except Exception as e:
        print(f"\n⚠️ Не удалось получить экстринсики: {e}")

print("\n" + "=" * 60)