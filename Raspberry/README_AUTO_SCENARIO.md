# Apex Rover AUTO Scenario

ضع مجلد `Auto` الناتج مكان مجلد:

```bash
/home/apex-rover/Apex_Rover/Auto
```

أو داخل مشروعك الحالي بجانب `Manual` و `main_startup.py`:

```text
Raspberry/
├── Auto/
│   ├── auto_config.py
│   ├── auto_io.py
│   ├── auto_status.py
│   ├── auto_stair_climb.py
│   ├── auto_vision.py
│   └── front_camera_server.py
├── Manual/
├── main_startup.py
└── src/
```

## صور src

أضف صور المجسمات داخل:

```bash
/home/apex-rover/Apex_Rover/src
```

أسماء مقترحة:

```text
source_box_1.jpg
stairs_1.jpg
destination_box_1.jpg
object_1.jpg
object_2.jpg
object_3.jpg
robot_basket.jpg
```

الأسماء مهمة لأن الكود يصنف الصور حسب الاسم.

## تشغيل Auto

شغل مدير الوضعيات كالمعتاد:

```bash
cd /home/apex-rover/Apex_Rover
python3 main_startup.py
```

ثم افتح:

```text
http://RASPBERRY_IP:5050/mode/auto
```

أو من شبكة ESP32:

```text
http://192.168.4.X:5050/mode/auto
```

## auto_status

أثناء Auto افتح:

```text
http://192.168.4.X:5000/auto_status
```

لأن `front_camera_server.py` في وضع Auto يعمل على port 5000 ويعرض التراك الشامل.

إذا بدك نفس endpoint على port 5050 داخل `main_startup.py`، أضف السطور الموجودة في:

```text
optional_main_startup_auto_status_route.py
```

## ملاحظات مهمة

- الكود لا يفتح Serial مع Mega.
- الحساسات تقرأ من `/tmp/apex_last_sensor.json` الذي يكتبه `Manual/sensor_bridge.py`.
- أوامر الحركة والجكات والذراع والكاميرا ترسل إلى ESP32 عبر:

```text
http://192.168.4.1/command?cmd=...
```

- الذراع يحتاج معايرة زوايا حقيقية في `auto_config.py` لأن كل روبوت مختلف ميكانيكياً.
- Ultrasonic مستخدم فقط كحماية stop/caution، وليس كأساس قرار الصعود.
- صعود الدرج يستخدم كاميرا + MPU، والجك الخلفي يتم تشغيله حتى يظهر تأثير على pitch/roll ثم يتوقف مع timeout حماية.
