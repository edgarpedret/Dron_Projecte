from pymavlink import mavutil
import time

def connect():
    """Estableix la connexió amb el dron."""
    print("Intentant connectar...")
    connection = mavutil.mavlink_connection('udpin:localhost:14551')
    connection.wait_heartbeat()  # Espera el senyal de vida del dron
    print(f"Connectat al dron - Sistema {connection.target_system}, Component {connection.target_component}")
    return connection

def set_mode(connection, mode):
    """Canvia el mode de vol del dron i espera confirmació."""
    mode_id = connection.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Mode {mode} no disponible")
        return False
    
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    print(f"Canviant a mode {mode}...")

    for _ in range(10):  # Intenta durant 10 segons
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            print(f"Mode {mode} activat")
            return True
        time.sleep(1)

    print("Error: No s'ha pogut canviar el mode de vol")
    return False

def arm_drone(connection):
    """Arma el dron i espera confirmació."""
    print("Armant motors...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

    for _ in range(10):  # Intenta durant 10 segons
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Dron armat amb èxit")
            return True
        time.sleep(1)

    print("Error: No s'ha pogut armar el dron")
    return False

def takeoff(connection, altitude):
    """Ordena l'enlairament del dron a una altitud específica."""
    if not set_mode(connection, "GUIDED"):  # Canvia al mode guiat
        return
    if not arm_drone(connection):  # Arma el dron abans d'enlairar-se
        return

    print("Enviant ordre d'enlairament...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )

    for _ in range(30):  # Intenta durant 30 segons
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # Converteix mm a metres
            print(f"Altitud actual: {current_alt:.2f}m")
            if current_alt >= altitude * 0.95:  # Si arriba al 95% de l'altitud desitjada
                print("Altitud d'enlairament assolida")
                return
        time.sleep(1)

    print("Error: No s'ha aconseguit assolir l'altitud d'enlairament")

def go_to_position(connection, lat, lon, alt):
    """Envia el dron a una posició GPS específica."""
    print(f"Anant a {lat}, {lon}, {alt}m")
    connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            10, connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            int(0b110111111000), int(lat * 10**7), int(lon * 10**7), alt,
            0, 0, 0, 0, 0, 0, 1.57, 0.5
        )
    )

def land(connection):
    """Ordena l'aterratge del dron."""
    print("Iniciant aterratge...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    for _ in range(30):  # Intenta durant 30 segons
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # Converteix mm a metres
            if current_alt <= 0.1:  # Si està gairebé aterrat
                print("Dron aterrat")
                return
        time.sleep(1)

    print("Error: No s'ha pogut aterrar correctament")

def main():
    """Funció principal que gestiona el menú de control del dron."""
    connection = connect()  # Connecta amb el dron
    
    while True:
        print("\n--- MENÚ DE CONTROL DEL DRON ---")
        print("1. Enlairar-se")
        print("2. Moure's a una ubicació")
        print("3. Aterrar")
        print("4. Sortir")
        
        opcion = input("Seleccioneu una opció: ")
        
        if opcion == "1":
            try:
                altitude = float(input("Introduïu l'altitud d'enlairament: "))
                takeoff(connection, altitude)
            except ValueError:
                print("Error: Introduïu un valor numèric vàlid.")
                
        elif opcion == "2":
            try:
                lat = float(input("Introduïu la latitud destí: "))
                lon = float(input("Introduïu la longitud destí: "))
                alt = float(input("Introduïu l'altitud destí: "))
                go_to_position(connection, lat, lon, alt)
            except ValueError:
                print("Error: Introduïu valors numèrics vàlids.")
        
        elif opcion == "3":
            land(connection)
        
        elif opcion == "4":
            print("Sortint...")
            break
        
        else:
            print("Opció no vàlida. Torneu-ho a intentar.")
    
if __name__ == "__main__":
    main()
