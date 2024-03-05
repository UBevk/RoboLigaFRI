#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Program za vodenje robota EV3 po seznamu tock na poligonu.
[Tekmovanje Robo liga FRI]
"""

__author__ = "Laboratory for adaptive systems and parallel processing"
__copyright__ = "Copyright 2023, UL FRI - LASPP"
__credits__ = ["Laboratory for adaptive systems and parallel processing"]
__license__ = "GPL"
__version__ = "0.2"
__maintainer__ = "Nejc Ilc"
__email__ = "nejc.ilc@fri.uni-lj.si"
__status__ = "Active"


# Če želite na svojem računalniku namestiti knjižnico python-ev3dev 
# in uprorabljati "code auto-completition":
# pip install python-ev3dev
#from ev3dev.ev3 import Button, LargeMotor, Sound
# Na EV3 robotu je potrebno namestiti paketa ujson in pycurl:
# sudo apt-get update
# sudo apt-get install python3-pycurl
# sudo apt-get install python3-ujson
# import pycurl
# import ujson
# import sys
import math
# from io import BytesIO
from time import time
# from enum import Enum
from collections import deque

from constants import *
from old_functions import *


# -----------------------------------------------------------------------------
# NASTAVITVE TIPAL, MOTORJEV IN POVEZAVE S STREŽNIKOM
# -----------------------------------------------------------------------------
# Nastavimo tipala in gumbe.
print('Priprava tipal ... ', end='', flush=True)


print('OK!')

# Nastavimo velika motorja. Priklopljena naj bosta na izhoda MOTOR_LEFT_PORT in MOTOR_RIGHT_PORT.
print('Priprava motorjev ... ', end='')
# FIXME: Začasna rešitev problema obrnjenih strani poligona - zamenjamo levi in desni motor
MOTOR_LEFT_PORT, MOTOR_RIGHT_PORT = MOTOR_RIGHT_PORT, MOTOR_LEFT_PORT

motor_left = init_large_motor(MOTOR_LEFT_PORT)
motor_right = init_large_motor(MOTOR_RIGHT_PORT)
print('OK!')

# Nastavimo povezavo s strežnikom.
url = SERVER_URL + GAME_ID
print('Vzpostavljanje povezave z naslovom ' + url + ' ... ', end='', flush=True)
conn = Connection(url)
print('OK!')

# Informativno izmerimo zakasnitev pri pridobivanju podatkov (povprečje num_iters meritev)
print('Zakasnitev v komunikaciji s streznikom ... ', end='', flush=True)
print('%.4f s' % (conn.test_delay(num_iters=10)))


# -----------------------------------------------------------------------------
# PRIPRAVA NA TEKMO
# -----------------------------------------------------------------------------
# Pridobimo podatke o tekmi.
game_state = conn.request()
# Ali naš robot sploh tekmuje? Če tekmuje, ali je rdeča ali modra ekipa?

if ROBOT_ID not in game_state['teams']:
    print(f'Robot v tekmi {game_state["id"]} ne tekmuje.')
    robot_die()

my_color = game_state['teams'][ROBOT_ID]['color']
print('Robot tekmuje v ekipi:', my_color)

# Določi cilje za robota (seznam točk na poligonu).
# Našem primeru se bo vozil po notranjih kotih obeh košar, vmes pa bo obiskal polnilno postajo
# Izračunajmo središče polnilne postaje 1
chrg_st_1 = game_state['fields']['charging_station_1']
chrg_st_1_center_x = (chrg_st_1['top_right']['x'] + chrg_st_1['top_left']['x']) / 2
chrg_st_1_center_y = (chrg_st_1['bottom_right']['y'] + chrg_st_1['top_right']['y']) / 2
chrg_st_1_center = Point({'x': chrg_st_1_center_x, 'y': chrg_st_1_center_y})

targets_list = [
    Point(game_state['fields']['blue_basket']['bottom_right']),
    Point(game_state['fields']['blue_basket']['top_right']),
    chrg_st_1_center,
    Point(game_state['fields']['red_basket']['top_left']),
    Point(game_state['fields']['red_basket']['bottom_left']),
]
print('Seznam ciljnih tock:')
for trgt in targets_list:
    print('\t' + str(trgt))

targets_labels = [
    'blue_basket_bottom_right',
    'blue_basket_top_right',
    'charging_station',
    'red_basket_top_left',
    'red_basket_bottom_left',
    ]

# -----------------------------------------------------------------------------
# GLAVNA ZANKA
# -----------------------------------------------------------------------------
print('Izvajam glavno zanko. Prekini jo s pritiskom na tipko DOL.')
print('Cakam na zacetek tekme ...')

# Začetno stanje.
state = State.IDLE
# Prejšnje stanje.
state_old = -1
# Indeks trenutne ciljne lokacije.
target_idx = 0

# Regulator PID za obračanje na mestu.
# setpoint=0 pomeni, da naj bo kot med robotom in ciljem (target_angle) enak 0.
# Naša regulirana veličina je torej kar napaka kota, ki mora biti 0.
# To velja tudi za regulacijo vožnje naravnost.
PID_turn = PID(
    setpoint=0,
    Kp=PID_TURN_KP,
    Ki=PID_TURN_KI,
    Kd=PID_TURN_KD,
    integral_limit=PID_TURN_INT_MAX)

# PID za vožnjo naravnost - regulira nazivno hitrost za oba motorja,
# ki je odvisna od oddaljenosti od cilja.
# setpoint=0 pomeni, da mora biti razdalja med robotom in ciljem enaka 0.
PID_frwd_base = PID(
    setpoint=0,
    Kp=PID_STRAIGHT_KP,
    Ki=PID_STRAIGHT_KI,
    Kd=PID_STRAIGHT_KD,
    integral_limit=PID_STRAIGHT_INT_MAX)

# PID za obračanje med vožnjo naravnost.
# setpoint=0 pomeni, da naj bo kot med robotom in ciljem (target_angle) enak 0.
PID_frwd_turn = PID(
    setpoint=0,
    Kp=PID_TURN_KP,
    Ki=PID_TURN_KI,
    Kd=PID_TURN_KD,
    integral_limit=PID_TURN_INT_MAX)

# Hitrost na obeh motorjih.
speed_right = 0
speed_left = 0

# Zgodovina (okno) zadnjih nekaj vrednosti meritev kota in razdalje.
# Implementirana je kot vrsta FIFO.
robot_dir_hist = deque([180.0] * HIST_QUEUE_LENGTH)
robot_dist_hist = deque([math.inf] * HIST_QUEUE_LENGTH)

# Merimo čas obhoda zanke. Za visoko odzivnost robota je zelo pomembno,
# da je ta čas čim krajši.
t_old = time()

do_main_loop = True
while do_main_loop and not btn.down:

    time_now = time()
    loop_time = time_now - t_old
    t_old = time_now

    # Zaznaj spremembo stanja.
    if state != state_old:
        state_changed = True
        print('Sprememba stanja, novo stanje:', state)
    else:
        state_changed = False
    state_old = state

    # Iz seznama ciljev izberi naslednjega.
    target = targets_list[target_idx]

    # Osveži stanje tekme.
    game_state = conn.request()
    if game_state == -1:
        print('Napaka v paketu, ponovni poskus ...')
    else:
        # Ali tekma teče?
        game_on = game_state['game_on']
        # Koliko časa je do konca tekme?
        time_left = game_state['time_left']
        # Koliko goriva ima še moj robot? (merjeno v času)
        fuel = game_state['teams'][ROBOT_ID]['fuel']
        # Za testiranje lahko to ignoriramo po francosko
        #fuel = 100

        # Pridobi pozicijo in orientacijo svojega robota
        if ROBOT_ID in game_state['robots']:
            robot_pos = Point(game_state['robots'][ROBOT_ID]['position'])
            robot_dir = game_state['robots'][ROBOT_ID]['dir']
            robot_data_valid = True
        else:
            # Sistem nima podatkov o našem robotu, morda ne zazna oznake na robotu.
            robot_data_valid = False

        # Če tekma poteka in so podatki robota na voljo in robot ima še kaj goriva,
        # potem izračunamo novo hitrost na motorjih.
        # Sicer motorje ustavimo.
        # TODO: lahko upoštevamo tudi game_paused
        if game_on and robot_data_valid and fuel > 0:
            # Razdalja med robotom in ciljem.
            target_dist = get_distance(robot_pos, target)
            # Kot med robotom in ciljem.
            target_angle = get_angle(robot_pos, robot_dir, target)

            # Spremljaj zgodovino meritev kota in oddaljenosti.
            # Odstrani najstarejši element in dodaj novega - princip FIFO.
            robot_dir_hist.popleft()
            robot_dir_hist.append(target_angle)
            robot_dist_hist.popleft()
            robot_dist_hist.append(target_dist)

            if state == State.IDLE:
                # Stanje mirovanja - tu se odločamo, kaj bo robot sedaj počel.
                speed_right = 0
                speed_left = 0
                                
                # Preverimo, ali je robot na ciljni točki;
                if target_dist > DIST_EPS:
                    # če ni, ga tja pošljemo -> gremo v stanje TURN
                    state = State.TURN
                    robot_near_target_old = False
                else:
                    # če je, naložimo naslednji cilj, razen ...
                    state = State.LOAD_NEXT_TARGET
                    # ... če je robot na polnilni postaji.
                    if targets_labels[target_idx] == 'charging_station':
                        # Počakaj do napolnjenosti.
                        if fuel < MAX_FUEL_RECHARGE_TIME:
                            state = State.IDLE                    

            elif state == State.LOAD_NEXT_TARGET:
                # Naložimo naslednjo ciljno točko iz seznama.
                target_idx = target_idx + 1
                # Če smo prišli do konca seznama, gremo spet od začetka
                if target_idx >= len(targets_list):
                    target_idx = 0
                print(targets_labels[target_idx])
                # Gremo v stanje IDLE, da preverimo, ali smo morda že kar na cilju.
                state = State.IDLE

            elif state == State.TURN:
                # Obračanje robota na mestu, da bo obrnjen proti cilju.
                if state_changed:
                    # Če smo ravno prišli v to stanje, najprej ponastavimo PID.
                    PID_turn.reset()

                # Ali smo že dosegli ciljni kot?
                # Zadnjih nekaj obhodov zanke mora biti absolutna vrednost
                # napake kota manjša od DIR_EPS.
                err = [abs(a) > DIR_EPS for a in robot_dir_hist]

                if sum(err) == 0:
                    # Vse vrednosti so znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    state = State.DRIVE_STRAIGHT
                else:
                    # Reguliramo obračanje.
                    # Ker se v regulatorju trenutna napaka izračuna kot:
                    #   error = setpoint - measurement,
                    # dobimo negativno vrednost, ko se moramo zavrteti
                    # v pozitivno smer.
                    # Primer:
                    #   Robot ima smer 90 stopinj (obrnjen je proti "severu").
                    #   Cilj se nahaja na njegovi levi in da ga doseže,
                    #   se mora obrniti za 90 stopinj.
                    #       setpoint=0
                    #       target_angle = measurement = 90
                    #       error = setpoint - measurement = -90
                    #       u = funkcija, odvisna od error in parametrov PID.
                    #   Če imamo denimo Kp = 1, Ki = Kd = 0, potem bo u = -90.
                    #   Robot se mora zavrteti v pozitivno smer,
                    #   torej z desnim kolesom naprej in levim nazaj.
                    #   Zato:
                    #   speed_right = -u
                    #   speed_left = u
                    #   Lahko bi tudi naredili droben trik in bi rekli:
                    #       measurement = -target_angle.
                    #   V tem primeru bi bolj intuitivno nastavili
                    #   speed_right = u in speed_left = -u.
                    u = PID_turn.update(measurement=target_angle)
                    speed_right = -u
                    speed_left = u

            elif state == State.DRIVE_STRAIGHT:
                # Vožnja robota naravnost proti ciljni točki.
                # Vmes bi radi tudi zavijali, zato uporabimo dva regulatorja.
                if state_changed:
                    # Ponastavi regulatorja PID.
                    PID_frwd_base.reset()
                    PID_frwd_turn.reset()
                    timer_near_target = TIMER_NEAR_TARGET

                # Ali smo blizu cilja?
                robot_near_target = target_dist < DIST_NEAR
                if not robot_near_target_old and robot_near_target:
                    # Vstopili smo v bližino cilja.
                    # Začnimo odštevati varnostno budilko.
                    timer_near_target = TIMER_NEAR_TARGET
                if robot_near_target:
                    timer_near_target = timer_near_target - loop_time
                robot_near_target_old = robot_near_target

                # Ali smo že na cilju?
                # Zadnjih nekaj obhodov zanke mora biti razdalja do cilja
                # manjša ali enaka DIST_EPS.
                err_eps = [d > DIST_EPS for d in robot_dist_hist]
                if sum(err_eps) == 0:
                    # Razdalja do cilja je znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    state = State.IDLE #State.LOAD_NEXT_TARGET
                elif timer_near_target < 0:
                    # Smo morda blizu cilja in je varnostna budilka potekla?
                    speed_right = 0
                    speed_left = 0
                    state = State.TURN
                else:
                    u_turn = PID_frwd_turn.update(measurement=target_angle)
                    # Ker je napaka izračunana kot setpoint - measurement in
                    # smo nastavili setpoint na 0, bomo v primeru u_base dobili
                    # negativne vrednosti takrat, ko se bo robot moral premikati
                    # naprej. Zato dodamo minus pri izračunu hitrosti motorjev.
                    u_base = PID_frwd_base.update(measurement=target_dist)
                    # Omejimo nazivno hitrost, ki je enaka za obe kolesi,
                    # da imamo še manevrski prostor za zavijanje.
                    u_base = min(max(u_base, -SPEED_BASE_MAX), SPEED_BASE_MAX)
                    speed_right = -u_base - u_turn
                    speed_left = -u_base + u_turn

            # Omejimo vrednosti za hitrosti na motorjih.
            speed_right = round(
                            min(
                                max(speed_right, -SPEED_MAX),
                                SPEED_MAX)
                            )
            speed_left = round(
                            min(
                                max(speed_left, -SPEED_MAX),
                                SPEED_MAX)
                            )

            # Izračunane hitrosti zapišemo na motorje.
            motor_right.run_forever(speed_sp=speed_right)
            motor_left.run_forever(speed_sp=speed_left)

        else:
            # Robot bodisi ni viden na kameri bodisi tekma ne teče, 
            # zato ustavimo motorje.
            motor_left.stop(stop_action='brake')
            motor_right.stop(stop_action='brake')

# Konec programa
robot_die()