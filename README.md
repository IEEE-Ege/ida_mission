# ida_mission — görev durum makinesi / mission state machine

**🇹🇷 [Türkçe](#türkçe) · 🇬🇧 [English](#english)**

---

## Türkçe

### Genel Bakış
P1/P2/P3 görev durum makinesi (FSM). Görev noktaları arasında Nav2'yi sürer,
turuncu kenar dubalarından **kapı (gate) hesaplar**, sanal koridor sınırı
yayınlar ve P3'te hedefe **görsel servo** (piksel hatası → yaw PID) uygular.
Durumlar: INIT / P1 / P2 / P3 / SAFE_MODE / CONNECTION_FAIL; `ida_safety` FDIR
sinyallerini ve YKİ heartbeat'ini kullanır.

### Kurulum
> Önkoşullar: ROS 2 Humble (+ Nav2), `colcon`, `rosdep`, Python + `pip`.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <REPO_URL> ida_mission   # ida_msgs'i de klonlayın
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/ida_mission/requirements.txt
colcon build --packages-select ida_mission
source install/setup.bash
```

### Kullanım
```bash
ros2 run ida_mission mission_node --ros-args --params-file \
  src/ida_mission/config/mission_params.yaml
```
> Çalıştırmadan önce `mission_params.yaml` içindeki `p1_waypoints` ve `p2_goal`
> koordinatlarını KENDİ değerlerinizle doldurun (varsayılanlar sıfır yer tutucu).

### Bağımlılıklar
ROS 2: `rclpy`, `nav2_msgs`, `action_msgs`, `mavros_msgs`, `ida_msgs` … Pip: `numpy`.

### Lisans
**MIT.** Bulaşıcı bağımlılık yoktur.

**Kullanım koşulları:** Özgürce kullanın/değiştirin/dağıtın; lisans bildirimini
koruyun. Geliştirme yaparsanız bize **PR açmanız bizi mutlu eder** (zorunlu değil).

### Özel veri
**Yarışma görev noktaları (P1 waypoints, P2 hedefi) kaldırılmıştır** — hem
`config/mission_params.yaml` hem de `mission_node.py` varsayılanları sıfır yer
tutucudur. Kapı genişliği eşikleri gibi genel algoritma parametreleri
korunmuştur.

---

## English

### Overview
P1/P2/P3 mission state machine (FSM). Drives Nav2 between waypoints, **computes
gates** from the orange boundary buoys, publishes a virtual-corridor boundary,
and performs **visual servoing** in P3 (pixel error → yaw PID). States: INIT /
P1 / P2 / P3 / SAFE_MODE / CONNECTION_FAIL; consumes `ida_safety` FDIR signals
and the ground-station heartbeat.

### Installation
> Prerequisites: ROS 2 Humble (+ Nav2), `colcon`, `rosdep`, Python + `pip`.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <REPO_URL> ida_mission   # also clone ida_msgs
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/ida_mission/requirements.txt
colcon build --packages-select ida_mission
source install/setup.bash
```

### Usage
```bash
ros2 run ida_mission mission_node --ros-args --params-file \
  src/ida_mission/config/mission_params.yaml
```
> Before running, fill `p1_waypoints` and `p2_goal` in `mission_params.yaml`
> with YOUR own coordinates (defaults are zero placeholders).

### Dependencies
ROS 2: `rclpy`, `nav2_msgs`, `action_msgs`, `mavros_msgs`, `ida_msgs` … Pip: `numpy`.

### License
**MIT.** No contagious dependency.

**Terms:** free to use/modify/distribute; preserve the license notice. If you
improve it, **a PR back to us would make us happy** (not required).

### Private data
The **competition waypoints (P1 waypoints, P2 goal) were removed** — defaults in
both `config/mission_params.yaml` and `mission_node.py` are zero placeholders.
Generic algorithm parameters (e.g. gate-width thresholds) are kept.

---

<div align="center">

💙 **Bu Repo IEEE Ege Mavi İnci İnsansız Deniz Aracı Takımı Yazılım Ekibi Tarafından Oluşturulmuştur, Yazılım Ekibimize Sevgilerle**

[@NightKnight-nx2](https://github.com/NightKnight-nx2) · [@yalinoner](https://github.com/yalinoner) · [@nilayyldz](https://github.com/nilayyldz)

</div>
