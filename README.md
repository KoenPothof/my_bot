# Robot navigatiemodule

Navigatiemodule voor een zorginstellingrobot, gebouwd op de EZ-Wheel SWD robotbasis
(ROS2 Iron + Nav2). De robot rijdt autonoom vooraf gedefinieerde routes, handelt
blokkades sociaal af (wachten → alternatief → terugkeren) en signaleert zijn gedrag
via knipperlichten en een buzzer. De voorspelbaarheid van de robot staat centraal.

## Nodes

| Node | Bestand | Functie |
|---|---|---|
| `patrol_node` | `my_bot/patrol_node.py` | State machine voor de patrouille. Stuurt waypoints naar Nav2 (`NavigateToPose`), handelt blokkades af en publiceert de status. |
| `indicator_node` | `my_bot/indicator_node.py` | Knipperlichten bij bochten (detectie via `/plan` + `/cmd_vel`), gevaarslichten bij stilstand, buzzerpiep bij geblokkeerd/gestopt/fout. |
| `environment_speed_node` | `my_bot/environment_speed_node.py` | Verlaagt de snelheid vóór scherpe bochten via `/speed_limit` (0.5 → 0.2 m/s). |
| `mqtt_hmi_bridge` | `my_bot/mqtt_hmi_bridge.py` | Brug tussen ROS2 en de FT2J HMI via MQTT (broker op de robot, poort 1883). |

## Belangrijkste topics

| Topic | Type | Richting | Doel |
|---|---|---|---|
| `/start_patrol` | `std_msgs/Bool` | → patrol | Route starten |
| `/stop_patrol` | `std_msgs/Bool` | → patrol | Route onderbreken |
| `/patrol_state` | `std_msgs/String` | patrol → | Status: `idle`, `rijdend`, `wachten`, `voltooid`, `gestopt`, `fout` |
| `/planner_selector` | `std_msgs/String` | patrol → Nav2 | Selecteert `GridBased` (plant om obstakels heen) i.p.v. EZ-Way's `Waypoint` (rechte lijn). QoS: transient_local. |
| `/indicators` | `std_msgs/String` | indicator → | `links`, `rechts`, `gevaar`, `uit` |
| `/buzzer` | `std_msgs/Bool` | indicator → | Buzzer aan/uit |
| `/speed_limit` | `nav2_msgs/SpeedLimit` | speed → Nav2 | Snelheidslimiet |

## MQTT topics (FT2J HMI)

Broker: Mosquitto op de robot, `10.1.0.2:1883` (HMI op `10.1.0.3` via ETH1).

| MQTT topic | Richting | Payload |
|---|---|---|
| `robot/start_patrol` | HMI → robot | `"1"` = starten |
| `robot/stop_patrol` | HMI → robot | `"1"` = stoppen |
| `robot/patrol_state` | robot → HMI | huidige status (string) |
| `robot/indicator_status` | robot → HMI | diagnostiek indicator_node |
| `robot/buzzer` | robot → HMI | `"1"` aan / `"0"` uit |

## Starten

Alles draait in de Docker container op de robot:

```bash
docker exec -it ez-way-v2-ros2-iron-amr bash
export CYCLONEDDS_URI=file:///home/ezway/.config/ez-way/ez_way_bringup/cyclonedds_config.xml
export ROS_LOG_DIR=/tmp
```

1. Zet de robot op **automatic mode** (start Nav2/AMCL via de EZ-Way software).
2. Start de eigen nodes:

```bash
ros2 run my_bot patrol_node.py
ros2 run my_bot indicator_node.py
ros2 run my_bot environment_speed_node.py
ros2 run my_bot mqtt_hmi_bridge.py
```

3. Start de route via de HMI of handmatig:

```bash
ros2 topic pub --once /start_patrol std_msgs/msg/Bool "{data: true}"
```

## Blokkade-afhandeling

1. Nav2 kan het doel niet bereiken → robot wacht **3 minuten** zodat mensen de
   doorgang kunnen vrijmaken (`wachten`).
2. Nog steeds geblokkeerd → alternatieve route via het volgende waypoint.
3. Alternatief ook geblokkeerd → robot keert terug naar het laatst bereikte
   waypoint en gaat naar `fout`; een operator moet ingrijpen.

## Kaarten

- **Navigeren**: AMCL lokaliseert op een vooraf opgenomen kaart.
- **Kaart maken**: zet de robot in SLAM-modus en rij hem rond met de controller;
  de kaart wordt real-time opgebouwd (SLAM Toolbox).
