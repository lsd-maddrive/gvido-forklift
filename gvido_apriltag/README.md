# apriltag_ros2

Этот репозиторий содержит ROS-узел (apriltag_ros) для обнаружения и оценки позы AprilTags – визуальных fiducial-маркеров
- [apriltag\_ros2](#apriltag_ros2)
  - [Репозитории для генерации маркеров Apriltag](#репозитории-для-генерации-маркеров-apriltag)
  - [Установка](#установка)
  - [Конвертация](#конвертация)
  - [Размеры бумаги для печати](#размеры-бумаги-для-печати)

## Репозитории для генерации маркеров Apriltag 

Этот репозиторий [AprilTag PDFs][] содержит ROS-узел (apriltag_ros) для обнаружения и оценки позы AprilTags – визуальных fiducial-маркеров

Этот репозиторий содержит предварительно сгенерированные PDF-файлы с AprilTag 3 маркерами. В настоящее время доступны маркеры семейства tag36h11 размерами 100 мм и 200 мм, подходящие для печати на бумаге форматов US Letter и A4.

tagsize: Размер маркера в метрах. Каждый маркер имеет черную и белую рамку, но в некоторых вариантах белая рамка находится внутри, а в других — снаружи. Параметр tagsize измеряется от места соединения двух рамок.

Исходный репозиторий изображений [AprilTag IMGs][]:

[AprilTag PDFs]: https://github.com/rgov/apriltag-pdfs/tree/main/tag36h11
[AprilTag IMGs]: https://github.com/AprilRobotics/apriltag-imgs

## Установка 

Выполните команду:

```bash
sudo apt install imagemagick
```

Для создания PDF может потребоваться изменить политику безопасности ImageMagick, разрешив запись в PDF.

Файл настроек:

```bash
/etc/ImageMagick-6/policy.xml
```

Закомментируйте строку:

```xml
<policy domain="coder" rights="none" pattern="PDF" />
```

## Конвертация

Пример команды для печати квадрата 250×250 мм на листе формата A3 (25.4 — коэффициент перевода дюймов в мм):

```bash
convert tag36_11_00001.png \
    -density 300 \
    -scale $((100 * 250./10 * 300/25.4))% \
    -bordercolor black -border 1 \
    -gravity center -extent $((300*11.7))x$((300*16.5)) \ 
    -gravity south -annotate +0+$((300*0.25)) 'AprilTag family = tag36h11, size = 250 mm, id = 1' \
    tag1_a3.pdf
```

## Размеры бумаги для печати

| Paper     | mm               | inches |
| :-------------:|:------------------:| :-----:|
| A3    | 	297 x 420 mm    | 11.7 x 16.5 inches |
| A4     | 210 x 297 mm |  	8.3 x 11.7 inches |
| A5  | 	148.5 x 210 mm        | 5.8 x 8.3 inches |
|  AGV markers  | 	550 x 550 mm        | 21.6535 x 21.6535 inches |
