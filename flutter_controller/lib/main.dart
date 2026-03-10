import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:app_settings/app_settings.dart';

void main() => runApp(const RobotApp());

class RobotApp extends StatelessWidget {
  const RobotApp({super.key});
  @override
  Widget build(BuildContext context) => MaterialApp(
    debugShowCheckedModeBanner: false,
    theme: ThemeData.dark(), 
    home: const ControlPanel()
  );
}

class ControlPanel extends StatefulWidget {
  const ControlPanel({super.key});
  @override
  State<ControlPanel> createState() => _ControlPanelState();
}

class _ControlPanelState extends State<ControlPanel> {
  double _speed = 600;
  final String espIp = "192.168.4.1"; 

  // Función principal para enviar comandos
  Future<void> _sendCommand(String dir) async {
    final url = Uri.parse('http://$espIp/move?dir=$dir&speed=${_speed.toInt()}');
    try {
      // Usamos un timeout corto para que la app no se congele si hay lag
      await http.get(url).timeout(const Duration(milliseconds: 300));
    } catch (e) {
      debugPrint("Error enviando $dir: $e");
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Robot Gesture Control"),
        actions: [
          IconButton(
            icon: const Icon(Icons.wifi),
            onPressed: () => AppSettings.openAppSettings(type: AppSettingsType.wifi),
          )
        ],
      ),
      body: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text("Velocidad PWM: ${_speed.toInt()}", 
               style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 30),
            child: Slider(
              value: _speed,
              min: 0, max: 1023,
              activeColor: Colors.orange,
              onChanged: (val) => setState(() => _speed = val),
            ),
          ),
          const SizedBox(height: 50),
          _buildJoystick(),
        ],
      ),
    );
  }

  // Widget que construye el D-Pad con detección de gestos
  Widget _buildJoystick() {
    return Column(
      children: [
        _movementKey(Icons.arrow_upward, "F"), // Adelante
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            _movementKey(Icons.arrow_back, "L"), // Izquierda
            const SizedBox(width: 80, height: 80), // Espacio central
            _movementKey(Icons.arrow_forward, "R"), // Derecha
          ],
        ),
        _movementKey(Icons.arrow_downward, "B"), // Atrás
      ],
    );
  }

  // Este es el componente clave: detecta cuando tocas y cuando sueltas
  Widget _movementKey(IconData icon, String cmd) {
    return GestureDetector(
      onTapDown: (_) => _sendCommand(cmd), // Al tocar: Inicia movimiento
      onTapUp: (_) => _sendCommand("S"),   // Al soltar: Detiene motores
      onTapCancel: () => _sendCommand("S"), // Si el gesto se interrumpe: Detiene
      child: Container(
        decoration: BoxDecoration(
          color: Colors.blueGrey.shade800,
          shape: BoxShape.circle,
          boxShadow: [
            BoxShadow(color: Colors.black26, blurRadius: 10, spreadRadius: 1)
          ],
        ),
        padding: const EdgeInsets.all(25),
        child: Icon(icon, size: 50, color: Colors.orangeAccent),
      ),
    );
  }
}