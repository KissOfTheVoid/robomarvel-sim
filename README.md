# Симуляция Robomarvel

Учебный симулятор к курсу «Интеллектуальная робототехника с ROS 2».

Симулируются три части робота: плата управления, лидар и источник видео. Одометрия,
построение карты, локализация и навигация работают штатными пакетами ROS 2 с теми же
настройками, что заданы на роботе, поэтому имена топиков, типы сообщений и структура систем
координат в симуляции и на роботе совпадают.

Теория, порядок сборки и пошаговые инструкции разобраны в материалах курса. Здесь собрано
только то, что нужно, чтобы поднять симулятор и убедиться, что он работает.

## Запуск контейнера

```bash
docker build -t robomarvel-sim .
docker run -it --name robomarvel-sim \
  -v "$(pwd)":/src \
  -p 127.0.0.1:8765:8765 \
  -p 127.0.0.1:8000:8000 \
  -p 127.0.0.1:8554:8554 \
  -p 127.0.0.1:8889:8889 \
  -p 127.0.0.1:8189:8189/udp \
  robomarvel-sim
```

Порты: `8765` — мост для средства визуализации, `8000` — VS Code в браузере,
`8554`, `8889`, `8189` — mediamtx для вебкамеры.

Повторный запуск уже созданного контейнера:

```bash
docker start -ai robomarvel-sim
```

## Рабочая папка и сборка

Рабочая папка в контейнере — `/src`, пакеты лежат в `/src/packages`. Так же устроен
репозиторий на роботе, поэтому команды и пути в материалах курса одинаковы для обеих сред.

Умолчания colcon заданы в `colcon-defaults.yaml`, поэтому достаточно набрать `colcon build`.
В контейнере есть сокращения: `ws` — переход в `/src`, `cb` — сборка, `cs` — подключение
собранного пакета к текущему терминалу.

```bash
python3 selfcheck.py     # проверка синтаксиса, YAML, URDF и расчётов, без запуска ROS
cb && cs
```

## Запуск системы

Вся симуляция поднимается одним launch-файлом:

```bash
ros2 launch robomarvel_sim sim.launch.py slam:=localization db:=/src/maps/classroom_good.db
```

| Аргумент | По умолчанию | Значения |
|---|---|---|
| `world` | `config/worlds/classroom.yaml` | путь к файлу планировки |
| `odom` | `icp` | `icp` — ICP-одометрия по сканам, `perfect` — истинная поза, для отладки |
| `slam` | `mapping` | `mapping` — строить карту, `localization` — работать по готовой, `off` — без карты |
| `db` | `/src/maps/rtabmap.db` | путь к базе rtabmap |
| `nav` | `true` | серверы Nav2 и `goal_proxy` |
| `detection` | `true` | виртуальная детекция объектов |

**Важно:** запуск с `slam:=mapping` удаляет базу при старте. Готовую карту храните отдельной
копией и указывайте её через `db:=`.

Карта сохраняется сама при остановке по Ctrl+C, отдельной команды для этого нет. Файл базы
и есть карта. Версия в формате `.pgm` и `.yaml` нужна только для просмотра, локализация ею
не пользуется:

```bash
ros2 run nav2_map_server map_saver_cli -f /src/maps/classroom
```

## Ноды пакета robomarvel_sim

| Нода | Что заменяет | Публикует |
|---|---|---|
| `sim_hwnode` | плату управления | `/hardware/status`, `/hardware/odom`, `/hardware/imu`, `/hardware/battery`, `/sim/ground_truth` |
| `sim_lidar` | драйвер лидара | `/scan`, 450 лучей, 10 Гц, система координат `lidar` |
| `sim_detection` | детектор объектов | `/detections`, `/camera/camera_info`, `/camera/image_annotated_compressed` |
| `goal_proxy` | та же нода работает на роботе | `/goal/status`, `/goal/feedback`, сервис `/goal/cancel` |

Остальное запускается штатными пакетами: `robot_state_publisher`, `icp_odometry`
из `rtabmap_odom`, `rtabmap` из `rtabmap_slam`, серверы Nav2 с менеджером жизненного цикла
и `foxglove_bridge`.

Трансформацию `odom` → `base_link` считает `icp_odometry` по симулированным сканам,
`sim_hwnode` её не публикует. Истинная поза уходит в отдельное дерево `sim_world` → `sim_base`,
не связанное с деревом робота, поэтому опереться на неё случайно нельзя. Разница между
`/icp/odom` и `/sim/ground_truth` — накопленный дрейф одометрии.

## Проверка установки

Ступени включают друг друга. Между запусками остановите предыдущий по Ctrl+C и убедитесь,
что `ros2 node list` пуст: два запущенных launch-файла дают дубли нод.

| № | Аргументы к `sim.launch.py` | Что должно получиться |
|---|---|---|
| 1 | `odom:=perfect slam:=off nav:=false detection:=false` | `/scan` около 10 Гц, `/hardware/status` около 50 Гц, `/hardware/battery` около 8.3 |
| 2 | `odom:=icp slam:=off nav:=false detection:=false` | `/icp/odom` около 10 Гц, в журнале строки `Odom: ratio=0.7-0.8` |
| 3 | `slam:=mapping nav:=false detection:=false` | `/map` прирастает при объезде помещения, база появляется в `maps/` |
| 4 | `slam:=localization db:=/src/maps/classroom_good.db nav:=false detection:=false` | `/map` появляется сразу целиком |
| 5 | `slam:=localization db:=/src/maps/classroom_good.db nav:=true detection:=false` | цель в `/goal` доводит робота до места |
| 6 | `slam:=localization db:=/src/maps/classroom_good.db nav:=true detection:=true` | `/detections` с классами объектов и дальностями |

Управление с клавиатуры для первых ступеней запускается во втором терминале после `cs`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Цель для пятой ступени можно отправить из терминала:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: -0.5}, orientation: {w: 1.0}}}"
ros2 topic echo /goal/status
```

## Средство визуализации

Откройте [Lichtblick](https://viz.robomarvel.ru) и подключитесь по адресу `ws://localhost:8765`.

Настройка панелей:

- 3D: системой координат сцены выберите `map`, включите топики `/scan`,
  `/global_costmap/costmap` и `/plan`. Топик `/map` при этом выключите, иначе карта закроет
  собой карту стоимости.
- Transforms: выключите `sim_world` и `sim_base`, они намеренно лежат в отдельном дереве
  и не связаны с `map`.
- Image: топик `/camera/image_annotated_compressed`.
- Отправка цели мышью: в настройках панели 3D в поле топика для публикации позы укажите
  `/goal`, штатное значение этого поля — `/move_base_simple/goal`.

## VS Code в браузере

В контейнере работает OpenVSCode Server: [http://localhost:8000](http://localhost:8000).
Запускается автоматически, журнал в `/tmp/openvscode.log`, рабочая папка — `/src`.

## Вебкамера

Пакет `webcam` принимает поток вебкамеры через mediamtx и публикует его в `/camera/image_raw`.

```bash
ros2 launch webcam webcam.yaml
```

Затем откройте страницу публикации потока и нажмите Publish:

[http://localhost:8889/cam/publish?video-codec=h264%2F90000&video-bitrate=1000&audio-device=none&audio-voice=false&video-width=1280&video-height=720&video-framerate=10](http://localhost:8889/cam/publish?video-codec=h264%2F90000&video-bitrate=1000&audio-device=none&audio-voice=false&video-width=1280&video-height=720&video-framerate=10)

Картинка появится в панели Image по топику `/camera/image_raw`. Разрешение, частота кадров
и сжатие задаются в `packages/webcam/launch/webcam.yaml`.

## Планировка помещения

Файл `config/worlds/classroom.yaml` описывает класс размером 8 × 6 метров: `walls` — отрезки
стен и перегородок, `boxes` — прямоугольная мебель, `circles` и `objects` — круглые
препятствия, `start` — точка старта робота. Свои помещения задаются такими же файлами.

## Ограничения

- **Звук.** Не реализован: Docker Desktop не пробрасывает `/dev/snd` в контейнер.
- **Точка старта при локализации.** Режим `localization` запускается с параметром
  `RGBD/StartAtOrigin=true`, то есть rtabmap считает, что робот стоит в начале карты.
  Для симуляции это верно, потому что старт берётся из файла планировки и карта строилась
  оттуда же. Если запустить робота в другом месте, трансформация `map` → `odom` останется
  неверной, пока rtabmap не узнает местность по сканам или пока ему не задать позу
  сообщением в `/initialpose`.
