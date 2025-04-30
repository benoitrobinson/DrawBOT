# Drawbot

<div align="center">
  <strong>Un robot autonome de dessin de précision</strong>
  <br>
  <br>
</div>

![Version](https://img.shields.io/badge/version-1.1-blue.svg) ![License](https://img.shields.io/badge/license-MIT-green.svg) ![Arduino Compatible](https://img.shields.io/badge/platform-ESP32-red.svg)

## 📋 Sommaire

- [Aperçu](#aperçu)
- [Fonctionnalités](#fonctionnalités)
- [Installation](#installation)
- [Utilisation](#utilisation)
- [Architecture](#architecture)
- [Documentation](#documentation)
- [Contribution](#contribution)
- [Licence](#licence)

## 🔍 Aperçu

Drawbot est une plateforme robotique mobile conçue pour réaliser des dessins de précision sur papier. Développé dans le cadre du cours "Systèmes Bouclés" à l'ECE Paris, ce projet implémente des algorithmes avancés de contrôle en boucle fermée pour garantir une haute précision dans les tracés.

## ✨ Fonctionnalités

- **Séquences de dessin préconfigurées**:
  - **L'escalier**: Trace des lignes droites avec des virages précis à 90°
  - **Le cercle**: Dessine des cercles parfaits de rayon paramétrable (2-20 cm)
  - **La rose des vents**: Produit une flèche ou une rose des vents orientée vers le Nord magnétique

- **Système de contrôle avancé**:
  - Correcteurs PID pour chaque axe de mouvement
  - Fusion de données multi-capteurs
  - Compensation automatique des erreurs

- **Interface utilisateur intuitive**:
  - Application web responsive
  - Contrôle sans fil via Wi-Fi
  - Feedback visuel en temps réel

## 💻 Installation

### Prérequis

- Arduino IDE (v1.8+)
- Bibliothèque ESP32 pour Arduino
- Un robot Drawbot assemblé

### Étapes d'installation

1. **Cloner le dépôt**
   ```bash
   git clone https://github.com/username/drawbot.git
   cd drawbot
   ```

2. **Installer les dépendances**
   ```bash
   # Avec le gestionnaire de bibliothèques Arduino
   arduino-cli lib install PID_v1
   arduino-cli lib install ESP32WiFi
   arduino-cli lib install Wire
   ```

3. **Configuration**
   ```bash
   # Copier et personnaliser le fichier de configuration
   cp firmware/config.example.h firmware/config.h
   # Éditer les paramètres selon vos besoins
   ```

4. **Compiler et téléverser**
   ```bash
   arduino-cli compile --fqbn esp32:esp32:nodemcu-32s firmware/drawbot
   arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:nodemcu-32s firmware/drawbot
   ```

## 🚀 Utilisation

### Connexion au robot

1. Mettez le robot sous tension
2. Connectez-vous au réseau Wi-Fi `Drawbot-XXXX` (XXXX étant l'ID unique du robot)
3. Ouvrez votre navigateur et accédez à `http://192.168.4.1`

### Commande du robot

L'interface web vous permet de:
- Sélectionner la séquence de dessin
- Ajuster les paramètres (rayon, longueur, etc.)
- Lancer et arrêter l'exécution
- Visualiser l'état des capteurs

## 🏗️ Architecture

### Matériel

- **Microcontrôleur**: NodeMCU ESP32
- **Capteurs**:
  - Centrale inertielle LSM6DS3
  - Magnétomètre LIS3MDL
  - Encodeurs à effet Hall pour les moteurs
- **Actionneurs**:
  - Motoréducteurs N20 (100 RPM)
  - Drivers de moteur DRV8837DSGR

