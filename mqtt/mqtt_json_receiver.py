import json
import paho.mqtt.client as mqtt
import logging

# Configurazione logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')

# Configurazione MQTT
BROKER_HOST = "broker.emqx.io"  # Broker pubblico Eclipse Mosquitto per test
BROKER_PORT = 1883
TOPIC = "test/sensors/data"         # Topic di test (stesso del publisher)
USERNAME = None                     # Non necessario per broker pubblico
PASSWORD = None                     # Non necessario per broker pubblico

def on_connect(client, userdata, flags, rc):
    """Callback chiamata quando il client si connette al broker"""
    if rc == 0:
        logging.info("Connesso al broker MQTT")
        # Sottoscrive al topic
        client.subscribe(TOPIC)
        logging.info(f"Sottoscritto al topic: {TOPIC}")
    else:
        logging.error(f"Errore di connessione: {rc}")

def on_message(client, userdata, msg):
    """Callback chiamata quando arriva un messaggio"""
    try:
        # Decodifica il messaggio
        message_str = msg.payload.decode('utf-8')
        logging.info(f"Messaggio ricevuto sul topic {msg.topic}")
        
        # Parsing JSON
        json_data = json.loads(message_str)
        
        # Processa i dati JSON
        process_json_message(json_data)
        
    except json.JSONDecodeError as e:
        logging.error(f"Errore nel parsing JSON: {e}")
        logging.error(f"Messaggio originale: {message_str}")
    except Exception as e:
        logging.error(f"Errore nel processare il messaggio: {e}")

def process_json_message(data):
    """Processa il messaggio JSON ricevuto"""
    logging.info("Dati JSON ricevuti:")
    print(json.dumps(data, indent=2, ensure_ascii=False))
    
    # Esempio di elaborazione dei dati
    if isinstance(data, dict):
        for key, value in data.items():
            logging.info(f"  {key}: {value}")
    
    # Qui puoi aggiungere la tua logica di elaborazione
    # Ad esempio, salvare in database, inviare notifiche, ecc.

def on_disconnect(client, userdata, rc):
    """Callback chiamata quando il client si disconnette"""
    if rc != 0:
        logging.warning("Disconnessione inaspettata dal broker")
        logging.info(rc)
    else:
        logging.info("Disconnesso dal broker")

def on_subscribe(client, userdata, mid, granted_qos):
    """Callback chiamata quando la sottoscrizione è confermata"""
    logging.info(f"Sottoscrizione confermata con QoS: {granted_qos}")

def main():
    # Crea il client MQTT
    client = mqtt.Client()
    
    # Configura le callback
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.on_subscribe = on_subscribe
    
    # Configura autenticazione se necessaria
    if USERNAME and PASSWORD:
        client.username_pw_set(USERNAME, PASSWORD)
    
    try:
        logging.info(f"Tentativo di connessione a {BROKER_HOST}:{BROKER_PORT}")
        # Connetti al broker
        client.connect(BROKER_HOST, BROKER_PORT, 60)
        
        # Avvia il loop per mantenere la connessione
        logging.info("Avvio del loop MQTT...")
        client.loop_forever()
        
    except KeyboardInterrupt:
        logging.info("Interruzione da tastiera ricevuta")
        client.disconnect()
    except Exception as e:
        logging.error(f"Errore nella connessione: {e}")
    finally:
        logging.info("Script terminato")

if __name__ == "__main__":
    main()