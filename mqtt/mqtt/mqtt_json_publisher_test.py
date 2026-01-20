import json
import paho.mqtt.client as mqtt
import time
import random
import logging
from datetime import datetime
import os
from pathlib import Path

# Configurazione logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')

# Configurazione MQTT - Broker pubblico gratuito
# BROKER_HOST = "broker.emqx.io"  # Broker pubblico Eclipse Mosquitto
BROKER_HOST = "localhost" 
BROKER_PORT = 1883
TOPIC = "test/sensors/data"  # Topic di test
CLIENT_ID = f"mqtt_publisher_{random.randint(1000, 9999)}"


def read_data_from_file(file_name: str):
    """Legge dati da un file CSV usando la base risolta sopra."""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(base_dir, 'data', file_name)
    logging.info(f"Caricamento CSV da: {file_path}")
    with open(file_path, 'r') as file:
        data = file.readlines()

    header = data[0].strip().split(',')
    data = [line.strip() for line in data]
    if header and header[0] == '':
        header = header[1:]
        data = [line.split(',', 1)[1] for line in data[1:]]

    rows = [dict(zip(header, line.split(','))) for line in data[1:]]
    return rows

heartrate_data = read_data_from_file('AL_heartRate_data.csv')
rawdata = read_data_from_file('AL_rawdata.csv')
angles_data = read_data_from_file('AL_rawdata_with_angles.csv')

def on_connect(client, userdata, flags, rc):
    """Callback chiamata quando il client si connette al broker"""
    if rc == 0:
        logging.info("Publisher connesso al broker MQTT")
    else:
        logging.error(f"Errore di connessione publisher: {rc}")

def on_publish(client, userdata, mid):
    """Callback chiamata quando un messaggio è stato pubblicato"""
    logging.info(f"Messaggio pubblicato con ID: {mid}")

def generate_sensor_data():
    """Genera dati di esempio per simulare sensori"""
    sensors_data = {
        "timestamp": datetime.now().isoformat(),
        "device_id": f"sensor_{random.randint(1, 5)}",
        "location": random.choice(["Kitchen", "Living Room", "Bedroom", "Bathroom"]),
        "temperature": round(random.uniform(18.0, 28.0), 2),
        "humidity": round(random.uniform(30.0, 80.0), 2),
        "pressure": round(random.uniform(980.0, 1020.0), 2),
        "battery_level": random.randint(20, 100),
        "status": random.choice(["online", "maintenance", "warning"]),
        "readings": {
            "light": random.randint(0, 1000),
            "motion": random.choice([True, False]),
            "air_quality": random.choice(["good", "moderate", "poor"])
        }
    }
    return sensors_data

def generate_alert_data():
    """Genera dati di allerta"""
    alert_data = {
        "timestamp": datetime.now().isoformat(),
        "alert_type": random.choice(["temperature_high", "humidity_low", "battery_low", "sensor_offline"]),
        "severity": random.choice(["low", "medium", "high", "critical"]),
        "device_id": f"sensor_{random.randint(1, 5)}",
        "message": "Alert condition detected",
        "value": round(random.uniform(0, 100), 2),
        "threshold": round(random.uniform(50, 90), 2)
    }
    return alert_data

def generate_system_info():
    """Genera informazioni di sistema"""
    system_info = {
        "timestamp": datetime.now().isoformat(),
        "system_id": "mqtt_test_system",
        "version": "1.0.0",
        "uptime_seconds": random.randint(3600, 86400),
        "connected_devices": random.randint(5, 25),
        "message_count": random.randint(1000, 5000),
        "memory_usage": round(random.uniform(30.0, 80.0), 2),
        "cpu_usage": round(random.uniform(10.0, 90.0), 2)
    }
    return system_info

def generate_real_data():
    message_types = [
        # ("sensor_data", generate_sensor_data),
        # ("alert", generate_alert_data),
        # ("system_info", generate_system_info),
        ("heartrate_data", lambda: random.choice(heartrate_data)),
        # ("raw_data", lambda: random.choice(rawdata)),
        ("angles_data", lambda: random.choice(angles_data))
    ]
    
    msg_type, generator_func = random.choice(message_types)
            
    # Genera i dati
    data = generator_func()
    data["message_type"] = msg_type
            
    return data

def publish_test_messages(client):
    """Pubblica diversi tipi di messaggi di test"""
    
    message_types = [
        # ("sensor_data", generate_sensor_data),
        # ("alert", generate_alert_data),
        # ("system_info", generate_system_info),
        ("heartrate_data", lambda: random.choice(heartrate_data)),
        # ("raw_data", lambda: random.choice(rawdata)),
        ("angles_data", lambda: random.choice(angles_data))
    ]
    
    try:
        for i in range(20):  # Invia 20 messaggi
            # Scegli un tipo di messaggio casualmente
            msg_type, generator_func = random.choice(message_types)
            
            # Genera i dati
            data = generator_func()
            data["message_type"] = msg_type
            data["sequence_number"] = i + 1
            
            # Converti in JSON
            json_message = json.dumps(data, indent=None, ensure_ascii=False)
            
            # Pubblica il messaggio
            result = client.publish(TOPIC, json_message, qos=1)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logging.info(f"Messaggio {i+1} ({msg_type}) inviato")
                print(f"📤 Dati inviati: {json_message[:100]}...")
            else:
                logging.error(f"Errore nell'invio del messaggio {i+1}")
            
            # Attendi tra i messaggi
            time.sleep(2)
            
    except KeyboardInterrupt:
        logging.info("Interruzione da tastiera ricevuta")

def publish_single_test_message(client, message_data=None):
    """Pubblica un singolo messaggio di test personalizzato"""
    if message_data is None:
        message_data = {
            "timestamp": datetime.now().isoformat(),
            "test": True,
            "message": "Questo è un messaggio di test",
            "data": {
                "value1": 42,
                "value2": "hello world",
                "value3": [1, 2, 3, 4, 5]
            }
        }
    
    json_message = json.dumps(message_data, indent=2, ensure_ascii=False)
    result = client.publish(TOPIC, json_message, qos=1)
    
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        logging.info("Messaggio di test personalizzato inviato")
        print(f"📤 Messaggio inviato:\n{json_message}")
    else:
        logging.error("Errore nell'invio del messaggio di test")

def stream_online_mode(client, angles, heartrates, publish_delay=0.5):
    """
    Stream mode: publish all lines from the larger file (primary) and interleave
    lines from the smaller file every N primary messages so both finish roughly at same time.
    """
    import math

    len_angles = len(angles)
    len_heartrate = len(heartrates)

    if len_angles == 0 and len_heartrate == 0:
        logging.warning("No data available for online streaming.")
        return

    # Choose primary (larger) and secondary (smaller)
    if len_angles >= len_heartrate:
        primary = angles
        secondary = heartrates
        primary_type = "angles_data"
        secondary_type = "heartrate_data"
    else:
        primary = heartrates
        secondary = angles
        primary_type = "heartrate_data"
        secondary_type = "angles_data"

    # Avoid division by zero
    if len(secondary) == 0:
        N = 1
    else:
        N = math.ceil(len(primary) / len(secondary))

    logging.info(f"Streaming online mode: primary={primary_type} ({len(primary)}), "
                 f"secondary={secondary_type} ({len(secondary)}), N={N}")

    sec_index = 0
    seq = 1

    for i, item in enumerate(primary):
        # prepare and publish primary item
        msg = dict(item)  # copy
        msg["message_type"] = primary_type
        msg["sequence_number"] = seq
        json_message = json.dumps(msg, ensure_ascii=False)
        client.publish(TOPIC, json_message, qos=1)
        logging.info(f"Published primary [{seq}] {primary_type}")
        seq += 1
        time.sleep(publish_delay)

        # every N primary messages publish one secondary (if available)
        if (i + 1) % N == 0 and sec_index < len(secondary):
            sec_msg = dict(secondary[sec_index])
            sec_msg["message_type"] = secondary_type
            sec_msg["sequence_number"] = seq
            json_sec = json.dumps(sec_msg, ensure_ascii=False)
            client.publish(TOPIC, json_sec, qos=1)
            logging.info(f"Published secondary [{seq}] {secondary_type}")
            seq += 1
            sec_index += 1
            time.sleep(publish_delay)

    # If any secondary items remain (due to rounding), publish them at end spaced out
    while sec_index < len(secondary):
        sec_msg = dict(secondary[sec_index])
        sec_msg["message_type"] = secondary_type
        sec_msg["sequence_number"] = seq
        client.publish(TOPIC, json.dumps(sec_msg, ensure_ascii=False), qos=1)
        logging.info(f"Published remaining secondary [{seq}] {secondary_type}")
        seq += 1
        sec_index += 1
        time.sleep(publish_delay)


def main():
    """Funzione principale"""
    print("🚀 MQTT JSON Publisher per Test")
    print(f"📡 Broker: {BROKER_HOST}:{BROKER_PORT}")
    print(f"📋 Topic: {TOPIC}")
    print("=" * 50)
    
    # Crea il client MQTT
    client = mqtt.Client(client_id=CLIENT_ID)
    client.on_connect = on_connect
    client.on_publish = on_publish
    
    try:
        # Connetti al broker
        logging.info(f"Connessione a {BROKER_HOST}:{BROKER_PORT}")
        client.connect(BROKER_HOST, BROKER_PORT, 60)
        
        # Avvia il loop in background
        client.loop_start()
        
        # Attendi la connessione
        time.sleep(2)
        
        print("\nScegli un'opzione:")
        print("1. Invia messaggi di test automatici (20 messaggi)")
        print("2. Invia un singolo messaggio di test")
        print("3. Modalità interattiva (premi Enter per inviare)")
        print("4. Modalità online (stream synchronized heartrate & angles)")
        
        choice = input("Inserisci la tua scelta (1-4): ").strip()
        
        if choice == "1":
            print("\n🔄 Invio messaggi automatici...")
            publish_test_messages(client)
            
        elif choice == "2":
            print("\n📤 Invio messaggio singolo...")
            publish_single_test_message(client)
            
        elif choice == "3":
            print("\n⌨️  Modalità interattiva - premi Enter per inviare un messaggio (Ctrl+C per uscire)")
            counter = 1
            while True:
                input("Premi Enter per inviare un messaggio...")
                data = generate_real_data()
                data["interactive_message"] = counter
                publish_single_test_message(client, data)
                counter += 1

        elif choice == "4":
            print("\n🌐 Modalità online: streaming synchronized heartrate & angles")
            # use the preloaded lists: heartrate_data and angles_data
            stream_online_mode(client, angles=angles_data, heartrates=heartrate_data, publish_delay=0.05)
            
        else:
            print("Scelta non valida")
            
    except KeyboardInterrupt:
        print("\n🛑 Interruzione ricevuta")
    except Exception as e:
        logging.error(f"Errore: {e}")
    finally:
        client.loop_stop()
        client.disconnect()
        print("✅ Publisher disconnesso")

if __name__ == "__main__":
    main()