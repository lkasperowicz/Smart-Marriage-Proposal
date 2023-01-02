import os
from time import sleep, time
import shutil
import json
from datetime import datetime
from enum import Enum
from random import randint
from threading import Thread, Lock, Event
import base64
import serial
import RPi.GPIO as GPIO
from picamera import PiCamera
from web3 import Web3
from eth_utils import encode_hex

from proposal_art import ProposalArt
from smart_proposal_notification import EventManager

dir_path = os.path.dirname(os.path.realpath(__file__))

PROPOSAL_INTERFACE = "/dev/ttyUSB0"
SMART_PROPOSAL_CONTRACT_ADDRESS ='0xe4e7Ed4f714A8cbDA6Da05910484CFA796f992bb'
SMART_PROPOSAL_BOX_ADDRESS = '0x7C7B3A6C355da24fDF567d400A498a7e51315109'
SMART_PROPOSAL_CONTRACT_TRANSACTION_GAS_LIMIT = 70000
SMART_PROPOSAL_CONTRACT_TRANSACTION_GAS_FEE_LIMIT = 300
SMART_PROPOSAL_CONTRACT_TRANSACTION_PRIORITY_FEE_LIMIT = 2
PRIVATE_KEY = ''

camera = PiCamera()
proposal_accepted = False
proposal_rejected = False
location_verified = False
proposal_requested = False
proposal_finished = False
proposal_lat = 0
proposal_lon = 0
proposal_time = 0
video_file = ''
mutex = Lock()

with open(f"{dir_path}/secure/smart_proposal_box.key") as keyfile:
    PRIVATE_KEY = str(keyfile.read())

GPIO.setmode(GPIO.BOARD)

proposal_interface = serial.Serial(
        port=PROPOSAL_INTERFACE,
        timeout=1,
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS)

def SendSmartContractTransaction(transaction):
    retransmission = 5
    while retransmission > 0:
        try:
            transaction.update({'maxFeePerGas': w3.toWei(SMART_PROPOSAL_CONTRACT_TRANSACTION_GAS_FEE_LIMIT, 'gwei')})
            transaction.update({'maxPriorityFeePerGas': w3.toWei(SMART_PROPOSAL_CONTRACT_TRANSACTION_PRIORITY_FEE_LIMIT, 'gwei')})
            transaction.update({'nonce' : w3.eth.get_transaction_count(SMART_PROPOSAL_BOX_ADDRESS)})
            signed_transaction = w3.eth.account.sign_transaction(transaction, PRIVATE_KEY)
            transaction_hash = w3.eth.send_raw_transaction(signed_transaction.rawTransaction)
            print(f"Transaction hash: {encode_hex(transaction_hash)}")
            retransmission = 0
        except Exception as e:
            retransmission = retransmission - 1

def SendSmartContractVerifyLocationTransaction(lat, lon):
    print("SendSmartContractVerifyLocationTransaction")
    contract = w3.eth.contract(address=SMART_PROPOSAL_CONTRACT_ADDRESS, abi=ABI)
    transaction = contract.functions.verifyLocation(int(lat), int(lon)).buildTransaction()
    SendSmartContractTransaction(transaction)

def SendSmartContractSetProposalExpectedTimeTransaction(timestamp):
    print("SendSmartContractSetProposalExpectedTimeTransaction")
    contract = w3.eth.contract(address=SMART_PROPOSAL_CONTRACT_ADDRESS, abi=ABI)
    transaction = contract.functions.setProposalExpectedTime(int(timestamp)).buildTransaction()
    SendSmartContractTransaction(transaction)

def SendSmartContractSayYesTransaction(lat, lon, her_proof_hash, his_proof_hash):
    print("SendSmartContractSayYesTransaction")
    contract = w3.eth.contract(address=SMART_PROPOSAL_CONTRACT_ADDRESS, abi=ABI)
    transaction = contract.functions.sayYes(int(lat), int(lon), her_proof_hash, his_proof_hash).buildTransaction({'value': w3.toWei(0.0045, 'ether')})
    SendSmartContractTransaction(transaction)

def SendSmartContractSayNoTransaction():
    print("SendSmartContractSayNoTransaction")
    contract = w3.eth.contract(address=SMART_PROPOSAL_CONTRACT_ADDRESS, abi=ABI)
    transaction = contract.functions.sayNo().buildTransaction()
    SendSmartContractTransaction(transaction)

def SendSetProposalArtHash(art_hash):
    print("SendSetProposalArtHash")
    contract = w3.eth.contract(address=SMART_PROPOSAL_CONTRACT_ADDRESS, abi=ABI)
    transaction = contract.functions.setProposalArtHash(art_hash).buildTransaction()
    SendSmartContractTransaction(transaction)

def GenerateProposalTime(minimum_from_now, maximum_from_now):
    return time() + randint(minimum_from_now, maximum_from_now)

def TakeProposalPhotos():
    her_proof_hash = ""
    his_proof_hash = ""
    for x in range(10):
        now = datetime.now()
        dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
        file = f"/home/pi/camera/smart_proposal_{dt_string}.jpg"
        print("Smile!")
        camera.capture(file)
        if x == 0:
            #save source file for proposal art
            shutil.copyfile(file, "/home/pi/camera/proposal_art_source.jpg")
        elif x == 3:
            #save fist as her proof of proposal
            shutil.copyfile(file, "/home/pi/camera/her_proof.jpg")
        elif x == 9:
            #save last as his proof of proposal
            shutil.copyfile(file, "/home/pi/camera/his_proof.jpg")
        sleep(0.1)
    with open("/home/pi/camera/her_proof.jpg", "rb") as her_proof:
        her_proof_string = base64.b64encode(her_proof.read())
        # double hash
        her_proof_hash = Web3.keccak(Web3.keccak(her_proof_string))
    with open("/home/pi/camera/his_proof.jpg", "rb") as his_proof:
        his_proof_string = base64.b64encode(his_proof.read())
        # double hash
        his_proof_hash = Web3.keccak(Web3.keccak(his_proof_string))

    return her_proof_hash, his_proof_hash

def RecordProposalVideo():
    global video_file
    now = datetime.now()
    dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
    video_file = f"/home/pi/camera/smart_proposal_{dt_string}.h264"
    camera.start_recording(video_file)

def StopProposalVideo():
    global video_file
    camera.stop_recording()
    try:
        shutil.copyfile(video_file, "/home/pi/camera/common_proof.h264")
    except:
        pass

def SendProposalRequest():
    print("SendProposalRequest")
    proposal_interface.write(b"proposal_request\n") 

def ExecuteCurrentLocationCmd(message):
    global location_verified
    payload = json.loads(str(message))
    lat = float(payload.get('lat', 0)) * 100000
    lon = float(payload.get('lon', 0)) * 100000
    print(f"ParseCurrentLocationCmd: lat: {lat} lon: {lon}")
    if location_verified is False:
        try:
            SendSmartContractVerifyLocationTransaction(lat, lon)
            location_verified = True
        except:
            print("Contract transaction error")
            location_verified = False 

def ExecuteProposalAcceptedCmd(message):
    global proposal_accepted
    global proposal_lat
    global proposal_lon
    payload = json.loads(str(message))
    lat = float(payload.get('lat', 0)) * 100000
    lon = float(payload.get('lon', 0)) * 100000
    print(f"ExecuteProposalAcceptedCmd: lat: {lat} lon: {lon}")
    if proposal_accepted is False:
        proposal_accepted = True
        sleep(1)
        StopProposalVideo()
        her_proof_hash, his_proof_hash = TakeProposalPhotos()
        retry = 3600
        while retry > 0:
            try:
                SendSmartContractSayYesTransaction(lat, lon, her_proof_hash, his_proof_hash)
                retry = 0
            except:
                retry = retry - 1
            sleep(1)
        proposal_lat = lat
        proposal_lon = lon

def ExecuteProposalRejectedCmd(message):
    global proposal_rejected
    if proposal_rejected is False:
        proposal_rejected = True
        SendSmartContractSayNoTransaction()

def ExecuteGeofenceLocationCmd(message):
    payload = json.loads(str(message))
    lat = float(payload.get('lat', 0)) * 100000
    lon = float(payload.get('lon', 0)) * 100000
    print(f"ExecuteGeofenceLocationCmd: lat: {lat} lon: {lon}")
    
def ParseCmd(message):
    cmd_parser = {'current_location': ExecuteCurrentLocationCmd, 'proposal_accepted': ExecuteProposalAcceptedCmd, 'proposal_rejected': ExecuteProposalRejectedCmd, 'geofence_location': ExecuteGeofenceLocationCmd}
    mutex.acquire()
    try:
        payload = json.loads(str(message))
        cmd = payload.get('cmd', None)
        if cmd is not None:
            print(f"Execute command: {cmd}")
            cb = cmd_parser.get(cmd, None)
            if cb is not None:
             cb(message)
    except Exception as e:
        pass
    finally:
        mutex.release()

def CLILoop(proposal_request_event):
    while True:
        try:
            if proposal_request_event.is_set():
                SendProposalRequest()
                proposal_request_event.clear()
            line_string = proposal_interface.readline()
            line_string = line_string.strip().decode('utf-8')
        except Exception as e:
            print(f"proposal_interface.readline error: {e}")
            line_string = ""
        finally: 
            ParseCmd(line_string)

def HandleSmartContractLocationVerifiedEvent(args):
    retry = 3600
    while retry > 0:
        mutex.acquire()
        try:
            proposal_expected_time = GenerateProposalTime(600, 900)
            SendSmartContractSetProposalExpectedTimeTransaction(proposal_expected_time)
            print(f"now: {time()}, proposal_expected_time: {proposal_expected_time}")
            retry = 0
        except:
            retry = retry - 1
        finally:
            mutex.release()
        sleep(1)

def HandleSmartContractProposalExpectedTimeEvent(args):
    global proposal_time
    timestamp = args.get('timestamp', None)
    if timestamp is None:
        return
    mutex.acquire()
    proposal_time = timestamp
    mutex.release()

def HandleSmartContractProposalEvent(args):
    global proposal_finished
    mutex.acquire()
    proposal_finished = True
    mutex.release()

class EventListener:
    def __init__(self, handler):
        self._handler = handler

    def notify(self, args):
        self._handler(args)

camera.start_preview(alpha=192)

with open(f"{dir_path}/abi.json",'r') as abi_file:
    abi = abi_file.read()
ABI = json.loads(abi)
# HTTPProvider:
w3 = Web3(Web3.HTTPProvider("https://mainnet.infura.io/v3/<API-KEY"))
res = w3.isConnected()
if res is True:
    print("Web3 is connected")
else:
    print("Web3 is not connected")
w3.eth.defaultAccount = SMART_PROPOSAL_BOX_ADDRESS

listeners = [{'LocationVerifiedEvent': EventListener(HandleSmartContractLocationVerifiedEvent)},
            {'ProposalEvent': EventListener(HandleSmartContractProposalEvent)},
            {'ProposalExpectedTimeEvent': EventListener(HandleSmartContractProposalExpectedTimeEvent)},
            ]
contract = w3.eth.contract(address=SMART_PROPOSAL_CONTRACT_ADDRESS, abi=ABI)
contract_owner = contract.functions.getOwner().call()
print(f"Connection Test: Contract owner is {contract_owner}, my address is {SMART_PROPOSAL_BOX_ADDRESS}, I am owner: {SMART_PROPOSAL_BOX_ADDRESS==contract_owner}")
event_filters = []
event_filters.append(contract.events.LocationVerifiedEvent.createFilter(fromBlock='latest'))
event_filters.append(contract.events.ProposalEvent.createFilter(fromBlock='latest'))
event_filters.append(contract.events.ProposalExpectedTimeEvent.createFilter(fromBlock='latest'))
event_manager = EventManager(event_filters, listeners)
event_manager_thread = Thread(target=event_manager.loop, args=(1,), daemon=True)
event_manager_thread.start()

#startup delay
sleep(2)

proposal_request_event = Event()
cli_thread = Thread(target=CLILoop, daemon=True, args=(proposal_request_event,))
cli_thread.start()
#main loop
while True:
    mutex.acquire()
    try:
        if proposal_time > time():
            print(f"Time to proposal: {proposal_time - time()}")
        if proposal_requested is False and proposal_time != 0 and time() >= proposal_time:
            RecordProposalVideo()
            proposal_request_event.set()
            proposal_requested = True
    except:
        print("Proposal request error")
    finally:
        mutex.release()
    if proposal_finished is True:
        try:
            # create art at the end of proposal 
            line_size = randint(2, 5) + int(str(int(proposal_lat))[-1]) # some random and last digit of proposal lat as line size
            blur_value = randint(2, 5) + int(str(int(proposal_lon))[-1]) # some random and last digit of proposal lon as blur_value
            color_palette = randint(3, 5) + int(str(int(proposal_time))[-1]) # some random and last digit of proposal time as color_palette
            line_size = int(line_size / 2) * 2 + 1 # round to odd value
            blur_value = int(blur_value / 2) * 2 + 1 # round to odd value
            # line_size = 13
            # blur_value = 3
            # color_palette = 14
            print(f"Proposal_art inputs: l: {line_size}, b: {blur_value} c: {color_palette}")
            proposal_art = ProposalArt("/home/pi/camera/proposal_art_source.jpg")
            proposal_art.create("/home/pi/camera/proposal_art.jpg", line_size, blur_value, color_palette)
            with open("/home/pi/camera/proposal_art.jpg", "rb") as proposal_art_file:
                proposal_art_string = base64.b64encode(proposal_art_file.read())
                # double hash
                proposal_art_hash = Web3.keccak(Web3.keccak(proposal_art_string))
                mutex.acquire()
                SendSetProposalArtHash(proposal_art_hash)
                mutex.release()
                exit(0)

        except Exception as e:
            print(f"finish error: {e}")
            exit(1)
    sleep(1)


