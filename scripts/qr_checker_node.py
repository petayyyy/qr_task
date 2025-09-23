#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospkg
from flask import Flask, render_template, jsonify, request, send_file, redirect
import time
import threading
from queue import Queue
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode, ZBarSymbol
import csv
import logging
from datetime import datetime

# Try to import Clover service (if available)
try:
    from clover.srv import SetLEDEffect
except ImportError:
    rospy.logwarn("Clover package not available - LED effects disabled")

app = Flask(__name__)

# Get package path
rospack = rospkg.RosPack()
package_path = rospack.get_path('qr_checker')
www_path = os.path.join(package_path, 'www')

# Configure Flask template and static folders
app = Flask(__name__, 
            template_folder=os.path.join(www_path, 'templates'),
            static_folder=os.path.join(www_path, 'static'))

bridge = CvBridge()

# Disable Flask logging
# log = logging.getLogger('werkzeug')
# log.setLevel(logging.ERROR)

# Глобальные переменные для хранения статуса и результатов
qr_queue = Queue()
detection_active = True
ros_initialized = False
found_qrs = set()
max_qrs = 5
attempt_active = False
attempt_start_time = 0
attempt_duration = 0
all_found_flag = False
manual_qr_counter = 0
isFirstAtt = True
port_server = 5000

# Parameters for savig into file
isNeedName = True
participant_name = ""
qr_detection_times = []
users_file_name = os.path.join(package_path, 'qr_results.csv')

# LED system
isLed = False
try:
    set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
except rospy.ROSException as e:
    rospy.logerr(f"Failed to create LED service proxy: {e}")
    isLed = False

def format_number(number):
    if isinstance(number, (int, float)):
        return f"{number:.3f}".replace('.', ',')
    return number

def save_to_csv(filename: str):
    """Сохраняет результаты попытки в CSV файл"""
    global participant_name, qr_detection_times, attempt_duration, isFirstAtt, isNeedName
    
    if not isNeedName:
        participant_name = ""
    
    if isNeedName and not participant_name:
        rospy.logwarn("No participant name, skipping CSV save")
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
            
            row_data = {
                'timestamp': datetime.now().strftime("%d/%m/%y %H:%M"),
                'participant': participant_name if isNeedName else "",
                'completion_time': format_number(attempt_duration)
            }
            
            for i in range(max_qrs):
                if i < len(qr_detection_times) - 1:
                    row_data[f'qr_{i+1}_time'] = format_number(qr_detection_times[i])
                elif i < len(qr_detection_times):
                    row_data[f'qr_{i+1}_time'] = format_number(attempt_duration)
                else:
                    row_data[f'qr_{i+1}_time'] = ''
            
            writer.writerow(row_data)
        rospy.loginfo(f"Данные сохранены в файл: {filename}")
    except Exception as e:
        rospy.logerr(f"Error saving to CSV: {e}")

def setFound():
    global isLed
    if not isLed: 
        return
    try:
        set_effect(effect='blink', r=0, g=255, b=0)
        rospy.sleep(3)
        set_effect(r=0, g=0, b=0)
    except rospy.ServiceException as e:
        rospy.logwarn(f"LED service error: {e}")

def setEnd():
    global isLed
    if not isLed: 
        return
    
    try:
        for i in range(3):
            if i % 2 == 0: 
                set_effect(r=255, g=0, b=0)
            else: 
                set_effect(r=0, g=0, b=255)
            rospy.sleep(1)     
            set_effect(r=0, g=0, b=0)
            rospy.sleep(1)
    except rospy.ServiceException as e:
        rospy.logwarn(f"LED service error: {e}")

def setQr(qr_data:str):
    global detection_active, ros_initialized, found_qrs, attempt_active, all_found_flag, qr_detection_times, attempt_start_time, users_file_name
    
    if ("MANUAL_QR" in qr_data or (qr_data not in found_qrs)) and len(found_qrs) < max_qrs:
        detection_time = time.time() - attempt_start_time
        
        if len(found_qrs) < max_qrs-1:
            found_qrs.add(qr_data)
            qr_detection_times.append(detection_time)
            rospy.loginfo(f"Обнаружен новый QR-код: {qr_data} за {detection_time:.2f} секунд")
            qr_queue.put(qr_data)
            led_thread1 = threading.Thread(target=setFound)
            led_thread1.daemon = True
            led_thread1.start()
        elif len(found_qrs) == max_qrs-1:
            found_qrs.add(qr_data)
            qr_detection_times.append(detection_time)
            rospy.loginfo("Все QR-коды найдены!")
            all_found_flag = True
            attempt_active = False
            save_to_csv(users_file_name)
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
        barcodes = decode(img, symbols=[ZBarSymbol.QRCODE])
        
        if barcodes:
            for barcode in barcodes:
                qr_data = barcode.data.decode('utf-8')
                setQr(qr_data=qr_data)
    except Exception as e:
        rospy.logerr(f"Ошибка обработки изображения: {e}")

def init_ros():
    global image_sub, ros_initialized, max_qrs, port_server, isLed, isNeedName
    try:
        rospy.init_node('qr_checker', anonymous=True)

        max_qrs = rospy.get_param('~max_qrs', 5)
        port_server = rospy.get_param('~port', 5000)
        isLed = rospy.get_param('~led', True)
        isNeedName = rospy.get_param('~isNeedName', True)

        image_sub = rospy.Subscriber(rospy.get_param('~camera_topic', "main_camera/image_raw"), Image, image_callback)
        ros_initialized = True
        rospy.loginfo(f"Параметры запуска:\n Необходимое кол-во Qr:{max_qrs},\n Порт сервера:{port_server},\n Светодиодная индикация:{isLed},\n Необходимость ввода имени участника:{isNeedName}")
        rospy.loginfo("Node мониторинга QR запущена")
    except Exception as e:
        rospy.logerr(f"Ошибка инициализации ROS Node: {e}")

def reset_attempt():
    """Сброс попытки"""
    global found_qrs, attempt_active, attempt_start_time, attempt_duration, all_found_flag, manual_qr_counter, qr_detection_times
    found_qrs = set()
    attempt_active = False
    attempt_start_time = 0
    attempt_duration = 0
    all_found_flag = False
    manual_qr_counter = 0
    qr_detection_times = []

    rospy.loginfo("Попытка сброшена")

@app.route('/')
def index():
    return redirect('/qr_checker')

@app.route('/qr_checker')
def qr_checker():
    return render_template('qr_checker.html', isNeedName=isNeedName)

@app.route('/qr_status')
def qr_status():
    global attempt_duration, attempt_start_time, qr_detection_times, attempt_active, all_found_flag
    
    qr_codes = []
    all_found = False
    
    if attempt_active and attempt_start_time > 0:
        attempt_duration = time.time() - attempt_start_time
    
    while not qr_queue.empty():
        try:
            item = qr_queue.get_nowait()
            if item == "ALL_FOUND":
                all_found = True
                if attempt_active:
                    attempt_active = False
                    save_to_csv(users_file_name)
            else:
                qr_codes.append(item)
        except:
            break
    
    if len(found_qrs) >= max_qrs:
        all_found = True
        if attempt_active:
            attempt_active = False
            save_to_csv(users_file_name)
    
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
    global attempt_active, attempt_start_time, attempt_duration, all_found_flag, participant_name, qr_detection_times, users_file_name, isNeedName
    
    if not attempt_active:
        if isNeedName:
            name = request.json.get('participant_name', '') if request.json else ''
            if not name:
                return jsonify({'status': 'error', 'message': 'Не указано имя участника'})
            participant_name = name
        else:
            participant_name = ""
        
        reset_attempt()
        attempt_active = True
        attempt_start_time = time.time()
        attempt_duration = 0
        qr_detection_times = []
        rospy.loginfo(f"Попытка начата для участника: {participant_name if isNeedName else 'Без имени'}")
        return jsonify({'status': 'started', 'message': 'Попытка начата'})
    else:
        attempt_active = False
        attempt_duration = time.time() - attempt_start_time if attempt_start_time > 0 else 0
        save_to_csv(users_file_name)
        rospy.loginfo("Попытка остановлена")
        return jsonify({'status': 'stopped', 'message': 'Попытка остановлена'})

@app.route('/download_existing_csv')
def download_existing_csv():
    try:
        if not os.path.exists(users_file_name):
            return jsonify({'status': 'error', 'message': 'Файл не найден'}), 404
        
        return send_file(
            users_file_name,
            as_attachment=True,
            download_name=f"qr_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            mimetype='text/csv'
        )
    except Exception as e:
        rospy.logerr(f"Error downloading CSV: {e}")
        return jsonify({'status': 'error', 'message': 'Ошибка при скачивании файла'}), 500

@app.route('/manual_add_qr', methods=['POST'])
def manual_add_qr():
    global found_qrs, manual_qr_counter, all_found_flag
    
    if not attempt_active:
        return jsonify({'status': 'error', 'message': 'Попытка не активна'})
    
    if all_found_flag:
        return jsonify({'status': 'error', 'message': 'Все QR-коды уже найдены'})
    
    manual_qr_counter += 1
    qr_data = f"MANUAL_QR_{manual_qr_counter}"
    setQr(qr_data=qr_data)

    return jsonify({'status': 'success', 'message': f'Добавлен QR-код: {qr_data}'})

@app.route('/reset_attempt', methods=['POST'])
def reset_attempt_endpoint():
    reset_attempt()
    return jsonify({'status': 'success', 'message': 'Состояние сброшено'})

@app.route('/static/<path:filename>')
def serve_static(filename):
    return app.send_static_file(filename)

def main():
    global port_server
    # Initialize ROS in main thread
    init_ros()
    if (not ros_initialized):
        return

    # Start Flask app
    rospy.loginfo(f"Flask сервер запущен на порте {port_server}")
    app.run(host='0.0.0.0', port=port_server, debug=False, use_reloader=False)

if __name__ == '__main__':
    main()