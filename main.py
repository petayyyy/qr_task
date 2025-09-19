# -*- coding: utf-8 -*-
from flask import Flask, render_template, jsonify, request, send_file, redirect
import time
import threading
from queue import Queue
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
import csv
import os
from datetime import datetime
from clover.srv import SetLEDEffect

app = Flask(__name__)
bridge = CvBridge()
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Глобальные переменные для хранения статуса и результатов
qr_queue = Queue()
detection_active = True
ros_initialized = False
found_qrs = set()  # Множество для хранения уникальных QR-кодов
max_qrs = 5        # Максимальное количество QR-кодов для обнаружения
attempt_active = False  # Флаг активности попытки
attempt_start_time = 0  # Время начала попытки
attempt_duration = 0    # Длительность попытки
all_found_flag = False  # Флаг, что все QR-коды найдены
manual_qr_counter = 0   # Счётчик для ручного добавления QR-кодов
isFirstAtt = True

# Новые переменные для имени участника и времени обнаружения
require_name = True  # Флаг обязательности ввода имени
participant_name = ""  # Имя текущего участника
qr_detection_times = []  # Времена обнаружения QR-кодов
users_file_name = 'qr_results.csv'
# out systems
isLed = True
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

def format_number(number):
    if isinstance(number, (int, float)):
        return f"{number:.3f}".replace('.', ',')
    return number

def save_to_csv(filename: str):
    """Сохраняет результаты попытки в CSV файл"""
    global participant_name, qr_detection_times, attempt_duration, isFirstAtt
    if not participant_name:
        print("No participant name, skipping CSV save")
        return
    
    file_exists = os.path.isfile(filename)
    
    try:
        with open(filename, 'a', newline='', encoding='utf-8-sig') as csvfile:
            fieldnames = ['timestamp', 'participant', 'completion_time'] + [f'qr_{i+1}_time' for i in range(max_qrs)]
            if isFirstAtt: 
                csvfile.write("\n")
                isFirstAtt = False
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter=';')
            
            if not file_exists:
                writer.writeheader()
            
            # Формируем данные для записи
            row_data = {
                'timestamp': datetime.now().strftime("%d/%m/%y %H:%M"),
                'participant': participant_name,
                'completion_time': format_number(attempt_duration)
            }
            
            # Добавляем времена обнаружения QR-кодов
            for i in range(max_qrs):
                if i < len(qr_detection_times):
                    if (i != 0): row_data[f'qr_{i+1}_time'] = format_number(qr_detection_times[i] - qr_detection_times[i-1])
                    else: row_data[f'qr_{i+1}_time'] = format_number(qr_detection_times[i])
                else:
                    row_data[f'qr_{i+1}_time'] = ''
            
            writer.writerow(row_data)
        print(f"Данные сохранены в файл: {filename}")
        print("#=================================================================#")
    except Exception as e:
        print(f"Error saving to CSV: {e}")

def setFound():
    global isLed
    if (not isLed): return
    set_effect(effect='blink', r=0, g=255, b=0)
    rospy.sleep(3)
    set_effect(r=0, g=0, b=0)

def setEnd():
    global isLed
    if (not isLed): return
    
    for i in range(3):
        if (i % 2 == 0): set_effect(r=255, g=0, b=0)
        else: set_effect(r=0, g=0, b=255)
        rospy.sleep(1)     
        set_effect(r=0, g=0, b=0)
        rospy.sleep(1)

def setQr(qr_data:str):
    global detection_active, ros_initialized, found_qrs, attempt_active, all_found_flag, qr_detection_times, attempt_start_time
    
    # Добавляем только новые уникальные QR-коды
    if ("MANUAL_QR" in qr_data or (qr_data not in found_qrs)) and len(found_qrs) < max_qrs:
        detection_time = time.time() - attempt_start_time
        
        if (len(found_qrs) < max_qrs-1):
            found_qrs.add(qr_data)
            qr_detection_times.append(detection_time)
            print(f"Обнаружен новый QR-код: {qr_data} за {detection_time:.2f} секунд")
            qr_queue.put(qr_data)
            led_thread1 = threading.Thread(target=setFound)
            led_thread1.daemon = True
            led_thread1.start()
        elif (len(found_qrs) == max_qrs-1):
            found_qrs.add(qr_data)
            qr_detection_times.append(detection_time)
            print("Все QR-коды найдены!")
            all_found_flag = True
            qr_queue.put("ALL_FOUND")
            led_thread2 = threading.Thread(target=setEnd)
            led_thread2.daemon = True
            led_thread2.start()

def image_callback(data):
    global detection_active, ros_initialized, found_qrs, attempt_active, all_found_flag
    if not (detection_active and attempt_active and not all_found_flag):
        return
    
    try:
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        barcodes = pyzbar.decode(img)
        
        if barcodes:
            for barcode in barcodes:
                qr_data = barcode.data.decode('utf-8')
                setQr(qr_data=qr_data)
    except Exception as e:
        print(f"Ошибка обработки изображения: {e}")

def init_ros():
    global image_sub, ros_initialized
    """Инициализация ROS в основном потоке"""
    try:
        rospy.init_node('qr_detector', anonymous=True, disable_signals=True)
        image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
        ros_initialized = True
        print("ROS инициализирован успешно")
    except Exception as e:
        print(f"Failed to initialize ROS node: {e}")

def reset_attempt():
    """Сброс попытки"""
    global found_qrs, attempt_active, attempt_start_time, attempt_duration, all_found_flag, manual_qr_counter, qr_detection_times, participant_name
    found_qrs = set()
    attempt_active = False
    attempt_start_time = 0
    attempt_duration = 0
    all_found_flag = False
    manual_qr_counter = 0
    qr_detection_times = []
   
    print("Попытка сброшена")

@app.route('/')
def index():
    return redirect('/qr_checker')

@app.route('/qr_checker')
def qr_checker():
    return render_template('qr_checker.html', require_name=require_name)

@app.route('/qr_status')
def qr_status():
    """Проверяет наличие новых QR-кодов в очереди и возвращает статус"""
    global attempt_duration, attempt_start_time, qr_detection_times
    
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
    if (len(qr_codes) >= max_qrs):
        all_found = True
        
    # Создаем список статусов для каждого QR-кода
    qr_statuses = []
    for i in range(max_qrs):
        if i < len(found_qrs):
            detection_time = qr_detection_times[i] if i < len(qr_detection_times) else 0
            qr_statuses.append({
                "found": True, 
                "code": list(found_qrs)[i],
                "detection_time": detection_time
            })
        else:
            qr_statuses.append({
                "found": False, 
                "code": "",
                "detection_time": 0
            })
    
    return jsonify({
        'qr_codes': qr_codes,
        'qr_statuses': qr_statuses,
        'all_found': all_found,
        'found_count': len(found_qrs),
        'max_qrs': max_qrs,
        'attempt_active': attempt_active,
        'attempt_duration': attempt_duration,
        'participant_name': participant_name
    })

@app.route('/toggle_attempt', methods=['POST'])
def toggle_attempt():
    """Переключает состояние попытки (начать/остановить)"""
    global attempt_active, attempt_start_time, attempt_duration, all_found_flag, participant_name, qr_detection_times, users_file_name
    
    if not attempt_active:
        # Проверяем имя участника, если требуется
        if require_name:
            name = request.json.get('participant_name', '') if request.json else ''
            if not name:
                return jsonify({'status': 'error', 'message': 'Не указано имя участника'})
            participant_name = name  # Устанавливаем имя до сброса
        
        # Начинаем новую попытку (сбрасываем всё, кроме имени)
        reset_attempt()  # Внутри reset_attempt() больше не сбрасывает participant_name
        attempt_active = True
        attempt_start_time = time.time()
        attempt_duration = 0
        qr_detection_times = []
        print(f"Попытка начата для участника: {participant_name}")
        return jsonify({'status': 'started', 'message': 'Попытка начата'})
    else:
        # Останавливаем текущую попытку
        attempt_active = False
        attempt_duration = time.time() - attempt_start_time if attempt_start_time > 0 else 0
        
        # Сохраняем результаты
        save_to_csv(users_file_name)
        
        print("Попытка остановлена")
        return jsonify({'status': 'stopped', 'message': 'Попытка остановлена'})

@app.route('/download_existing_csv')
def download_existing_csv():
    """Отдает существующий CSV файл для скачивания"""
    try:
        # Проверяем существование файла
        if not os.path.exists(users_file_name):
            return jsonify({'status': 'error', 'message': 'Файл не найден'}), 404
        
        # Отправляем файл для скачивания
        return send_file(
            users_file_name,
            as_attachment=True,
            download_name=f"qr_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            mimetype='text/csv'
        )
    except Exception as e:
        print(f"Error downloading CSV: {e}")
        return jsonify({'status': 'error', 'message': 'Ошибка при скачивании файла'}), 500
       

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
    setQr(qr_data=qr_data)

    return jsonify({'status': 'success', 'message': f'Добавлен QR-код: {qr_data}'})

@app.route('/static/<path:filename>')
def serve_static(filename):
    """Сервис для обслуживания статических файлов"""
    return app.send_static_file(filename)

if __name__ == '__main__':
    ros_thread = threading.Thread(target=init_ros)
    ros_thread.daemon = True
    ros_thread.start()
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)