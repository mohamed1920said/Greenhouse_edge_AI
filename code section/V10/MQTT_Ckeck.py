import json
import time
import threading
import paho.mqtt.client as mqtt

BROKER_HOST = "192.168.4.1"
BROKER_PORT = 1883
TOPIC_DATA = "greenhouse/data"       # ESP32 -> Python
TOPIC_CMD  = "greenhouse/commands"   # Python -> ESP32

connected_event = threading.Event()

def on_connect(client, userdata, flags, rc, properties=None):
    # paho v2 callback signature (works with CallbackAPIVersion.VERSION2)
    if rc == 0:
        print("[OK] Connected to broker")
        connected_event.set()

        # Subscribe to data topic
        client.subscribe(TOPIC_DATA, qos=0)
        print(f"[SUB] {TOPIC_DATA}")

        # Publish one command message to ESP32
        cmd = {
            "mode": "MANUAL",
            "heater": 0,
            "fan": 0,
            "pump": 0,
            "humid": 0,
            "light": 0,
            "temp_low": 18,
            "temp_high": 26,
            "hum_low": 50,
            "lux_low": 200,
            "moist_low": 30
        }
        payload = json.dumps(cmd)
        info = client.publish(TOPIC_CMD, payload=payload, qos=0, retain=False)
        print(f"[PUB] {TOPIC_CMD} -> {payload} (mid={info.mid})")
    else:
        print(f"[ERR] Connect failed rc={rc}")

def on_message(client, userdata, msg):
    text = msg.payload.decode("utf-8", errors="replace")
    print(f"[MSG] {msg.topic} -> {text}")

def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print(f"[OK] Subscribed (mid={mid}, qos={granted_qos})")

def on_disconnect(client, userdata, rc, properties=None):
    print(f"[INFO] Disconnected rc={rc}")

def main():
    # For paho-mqtt >= 2.0
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="python_test_client")

    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    client.on_disconnect = on_disconnect

    print(f"[INFO] Connecting to {BROKER_HOST}:{BROKER_PORT} ...")
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
    client.loop_start()

    # Wait for connection
    if not connected_event.wait(timeout=8):
        print("[ERR] Timeout waiting for broker connection")
        client.loop_stop()
        return

    # Keep listening to incoming greenhouse/data messages
    print("[INFO] Listening for 30 seconds...")
    time.sleep(30)

    # Send AUTO mode before exit (optional)
    auto_cmd = {"mode": "AUTO"}
    client.publish(TOPIC_CMD, json.dumps(auto_cmd))
    print(f"[PUB] {TOPIC_CMD} -> {auto_cmd}")

    client.disconnect()
    client.loop_stop()
    print("[DONE] Test finished")

if __name__ == "__main__":
    main()