from flask import Flask, render_template, jsonify
import time
import threading
from queue import Queue
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar

# out system
from clover.srv import SetLEDEffect
import pigpio

app = Flask(__name__)
bridge = CvBridge()

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

# Глобальные переменные для хранения статуса и результатов
qr_queue = Queue()
detection_active = True
ros_initialized = False
found_qrs = set()  # Множество для хранения уникальных QR-кодов
max_qrs = 4        # Максимальное количество QR-кодов для обнаружения
attempt_active = False  # Флаг активности попытки
attempt_start_time = 0  # Время начала попытки
attempt_duration = 0    # Длительность попытки
all_found_flag = False  # Флаг, что все QR-коды найдены
manual_qr_counter = 0   # Счётчик для ручного добавления QR-кодов

# out systems
isLed = True
isSound = False
soundPin = 11
pi = pigpio.pi()
pi.set_mode(soundPin, pigpio.OUTPUT)

def setFound():
    global isLed, isSound, soundPin
    if (not isLed and not isSound): return
    if (isLed):
        set_effect(effect='blink', r=0, g=255, b=0)
    if (isSound):
        pi.write(soundPin, 1)
    
    rospy.sleep(3)
    
    if (isLed):
        set_effect(r=0, g=0, b=0)
    if (isSound):
        pi.write(soundPin, 0)

def setEnd():
    global isLed, isSound, soundPin
    if (not isLed and not isSound): return

    for i in range(5):
        if (isLed):
            if (i % 2 == 0): set_effect(r=255, g=0, b=0)
            else: set_effect(r=0, g=0, b=255)
        if (isSound):
            pi.write(soundPin, 1)
        rospy.sleep(1)
        
        if (isLed):
            set_effect(r=0, g=0, b=0)
        if (isSound):
            pi.write(soundPin, 0)
        rospy.sleep(1)

def setQr(qr_data:str):
    global detection_active, ros_initialized, found_qrs, attempt_active, all_found_flag
    # Добавляем только новые уникальные QR-коды
    if ("MANUAL_QR" in qr_data or (qr_data not in found_qrs)) and len(found_qrs) < max_qrs:
        if (len(found_qrs) < max_qrs):
            found_qrs.add(qr_data)
            print(f"Обнаружен новый QR-код: {qr_data}")
            # Добавляем в очередь
            qr_queue.put(qr_data)
            setFound()
        elif (len(found_qrs) == max_qrs-1):
            found_qrs.add(qr_data)
            print("Все QR-коды найдены!")
            all_found_flag = True
            qr_queue.put("ALL_FOUND")
            setEnd()

def image_callback(data):
    global detection_active, ros_initialized, found_qrs, attempt_active, all_found_flag
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    if (detection_active and attempt_active and not all_found_flag):
        barcodes = pyzbar.decode(img)
        
        if barcodes:
            for barcode in barcodes:
                qr_data = barcode.data.decode('utf-8')
                setQr(qr_data = qr_data)

def init_ros():
    global image_sub
    """Инициализация ROS в основном потоке"""
    global ros_initialized
    try:
        rospy.init_node('qr_detector', anonymous=True, disable_signals=True)
        image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
        ros_initialized = True
        print("ROS node initialized successfully")
        # rospy.spin()
    except Exception as e:
        print(f"Failed to initialize ROS node: {e}")

def qr_detection_loop():
    """Функция для детекции QR-кодов с использованием ROS"""
    global detection_active, ros_initialized, found_qrs, attempt_active, all_found_flag
    
    if not ros_initialized:
        print("ROS not initialized, skipping QR detection")
        return
    
    print("Детекция QR-кодов запущена...")
    
    while detection_active and not rospy.is_shutdown():
        try:
            # Получаем изображение из ROS
            img_msg = rospy.wait_for_message('main_camera/image_raw', Image, timeout=1.0)
            img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            
            # Детекция QR-кодов (только если попытка активна и не все найдены)
            if attempt_active and not all_found_flag:
                barcodes = pyzbar.decode(img)
                
                if barcodes:
                    for barcode in barcodes:
                        qr_data = barcode.data.decode('utf-8')
                        
                        # Добавляем только новые уникальные QR-коды
                        if qr_data not in found_qrs and len(found_qrs) < max_qrs:
                            found_qrs.add(qr_data)
                            print(f"Обнаружен новый QR-код: {qr_data}")
                            # Добавляем в очередь
                            qr_queue.put(qr_data)
                            
                            # Если найдены все QR-коды, останавливаем попытку
                            if len(found_qrs) == max_qrs:
                                print("Все QR-коды найдены!")
                                all_found_flag = True
                                qr_queue.put("ALL_FOUND")
            
            # Небольшая задержка для снижения нагрузки на CPU
            time.sleep(0.1)
        except rospy.ROSException as e:
            # Таймаут при ожидании сообщения - это нормально, продолжаем цикл
            if "timeout" in str(e):
                continue
            print(f"ROS error: {e}")
            time.sleep(1)
        except Exception as e:
            print(f"Ошибка детекции: {e}")
            time.sleep(1)
    
    print("Детекция QR-кодов остановлена")

def reset_attempt():
    """Сброс попытки"""
    global found_qrs, attempt_active, attempt_start_time, attempt_duration, all_found_flag, manual_qr_counter
    found_qrs = set()
    attempt_active = False
    attempt_start_time = 0
    attempt_duration = 0
    all_found_flag = False
    manual_qr_counter = 0
    print("Попытка сброшена")

@app.route('/qr_checker')
def qr_checker():
    return render_template('qr_checker.html')

@app.route('/qr_status')
def qr_status():
    """Проверяет наличие новых QR-кодов в очереди и возвращает статус"""
    global attempt_duration, attempt_start_time
    
    qr_codes = []
    all_found = False
    
    # Обновляем длительность попытки, если она активна
    if attempt_active and attempt_start_time > 0:
        attempt_duration = time.time() - attempt_start_time
    
    # Извлекаем все доступные QR-коды из очереди
    while not qr_queue.empty():
        try:
            item = qr_queue.get_nowait()
            if item == "ALL_FOUND":
                all_found = True
            else:
                qr_codes.append(item)
        except:
            break
    
    # Создаем список статусов для каждого QR-кода
    qr_statuses = []
    for i in range(max_qrs):
        if i < len(found_qrs):
            qr_statuses.append({"found": True, "code": list(found_qrs)[i]})
        else:
            qr_statuses.append({"found": False, "code": ""})
    
    return jsonify({
        'qr_codes': qr_codes,
        'qr_statuses': qr_statuses,
        'all_found': all_found,
        'found_count': len(found_qrs),
        'max_qrs': max_qrs,
        'attempt_active': attempt_active,
        'attempt_duration': attempt_duration
    })

@app.route('/toggle_attempt', methods=['POST'])
def toggle_attempt():
    """Переключает состояние попытки (начать/остановить)"""
    global attempt_active, attempt_start_time, attempt_duration, all_found_flag
    
    if not attempt_active:
        # Начинаем новую попытку
        reset_attempt()
        attempt_active = True
        attempt_start_time = time.time()
        attempt_duration = 0
        print("Попытка начата")
        return jsonify({'status': 'started', 'message': 'Попытка начата'})
    else:
        # Останавливаем текущую попытку
        attempt_active = False
        attempt_duration = time.time() - attempt_start_time if attempt_start_time > 0 else 0
        print("Попытка остановлена")
        return jsonify({'status': 'stopped', 'message': 'Попытка остановлена'})

@app.route('/manual_add_qr', methods=['POST'])
def manual_add_qr():
    """Ручное добавление QR-кода"""
    global found_qrs, manual_qr_counter, all_found_flag
    
    if not attempt_active:
        return jsonify({'status': 'error', 'message': 'Попытка не активна'})
    
    if all_found_flag:
        return jsonify({'status': 'error', 'message': 'Все QR-коды уже найдены'})
    
    # Генерируем уникальный QR-код для ручного добавления
    manual_qr_counter += 1
    qr_data = f"MANUAL_QR_{manual_qr_counter}"
    setQr(qr_data = qr_data)

    return jsonify({'status': 'success', 'message': f'Добавлен QR-код: {qr_data}'})

@app.route('/static/<path:filename>')
def serve_static(filename):
    """Сервис для обслуживания статических файлов"""
    return app.send_static_file(filename)

if __name__ == '__main__':
    # Инициализируем ROS в основном потоке
    # init_ros()
    ros_thread = threading.Thread(target=init_ros)
    ros_thread.daemon = True
    ros_thread.start()
    # # Запускаем детекцию QR-кодов в фоновом режиме
    # detection_thread = threading.Thread(target=qr_detection_loop)
    # detection_thread.daemon = True
    # detection_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)