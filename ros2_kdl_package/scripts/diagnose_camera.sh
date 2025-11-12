#!/bin/bash

echo "=== DIAGNOSTICA CAMERA GAZEBO ==="
echo ""

echo "1. Verifico se Gazebo è attivo..."
if pgrep -x "gz" > /dev/null || pgrep -x "ign" > /dev/null || pgrep -x "gzserver" > /dev/null; then
    echo "✓ Gazebo è in esecuzione"
else
    echo "✗ Gazebo NON è in esecuzione!"
    echo "  Lancia prima: ros2 launch ros2_kdl_package vision_demo.launch.py"
    exit 1
fi

echo ""
echo "2. Topic Gazebo Transport (gz topic -l)..."
if command -v gz &> /dev/null; then
    gz topic -l 2>/dev/null | grep -i camera || echo "Nessun topic camera trovato con 'gz'"
elif command -v ign &> /dev/null; then
    ign topic -l 2>/dev/null | grep -i camera || echo "Nessun topic camera trovato con 'ign'"
else
    echo "✗ Comando gz/ign non trovato"
fi

echo ""
echo "3. Topic ROS 2..."
ros2 topic list | grep -E '(wrist|camera|aruco)' || echo "Nessun topic camera/aruco in ROS 2"

echo ""
echo "4. Nodi attivi..."
ros2 node list | grep -E '(bridge|aruco|kdl)' || echo "Nessun nodo bridge/aruco/kdl"

echo ""
echo "5. Frequenza topic ROS 2 (10 secondi)..."
timeout 10s ros2 topic hz /wrist_camera/image 2>&1 || echo "Topic /wrist_camera/image non pubblica"

echo ""
echo "6. Verifica bridge..."
ros2 node info /wrist_camera_bridge 2>/dev/null || echo "Bridge non attivo"

echo ""
echo "=== FINE DIAGNOSTICA ==="
echo ""
echo "Se il topic Gazebo esiste ma ROS 2 non riceve nulla:"
echo "  1. Verifica sintassi bridge in vision_demo.launch.py"
echo "  2. Riavvia tutto dopo rebuild"
echo "  3. Controlla nome esatto del topic Gazebo (punto 2 sopra)"
