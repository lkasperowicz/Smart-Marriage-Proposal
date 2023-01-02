
import os
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.application import MIMEApplication
from web3 import Web3
from threading import Thread
import time
import json

dir_path = os.path.dirname(os.path.realpath(__file__))

SMART_PROPOSAL_CONTRACT_ADDRESS ='0xe4e7Ed4f714A8cbDA6Da05910484CFA796f992bb'

class ProposalGeofenceListener:
    def __init__(self, receiver = 'yourmail@mail.com'):
        self._sender_address = 'sendermail@gmail.com'
        with open(f"{dir_path}/secure/email_pass.txt") as passfile:
            self._sender_pass = str(passfile.read())
        self._receiver_address = receiver
        self._mail_content = '''
        Geofence Alert
        '''
        #Setup the MIME
        self._message = MIMEMultipart()
        self._message['From'] =  self._sender_address
        self._message['To'] = self._receiver_address
        self._message['Subject'] = 'Smart Proposal Blockchain Event: Geofence'
        self._message.attach(MIMEText(self._mail_content, 'plain'))

    def notify(self, args):
        #Create SMTP session for sending the mail
        session = smtplib.SMTP('smtp.gmail.com', 587) #use gmail with port
        session.starttls() #enable security
        session.login(self._sender_address, self._sender_pass) #login with mail_id and password
        text = self._message.as_string()
        session.sendmail(self._sender_address, self._receiver_address, text)
        session.quit()
        print('ProposalGeofenceListener Notification Sent')

class ProposalYesListener:
    def __init__(self, file_img = None, file_video = None, receiver = 'yourmail@mail.com'):
        self._sender_address = 'sendermail@gmail.com'
        with open(f"{dir_path}/secure/email_pass.txt") as passfile:
            self._sender_pass = str(passfile.read())
        self._receiver_address = receiver
        self._mail_content = '''
        Said Yes!!
        '''
        self._file_img = file_img
        self._file_video = file_video
        #Setup the MIME
        self._message = MIMEMultipart()
        self._message['From'] =  self._sender_address
        self._message['To'] = self._receiver_address
        self._message['Subject'] = 'Smart Proposal Blockchain Event: Yes'
        self._message.attach(MIMEText(self._mail_content, 'plain'))
    
    def notify(self, args):
        yes = args.get('yes', None)
        if yes is not True:
            return
        if self._file_img is not None:
            with open(self._file_img,'rb') as attach_file: # Open the file as binary mode
                image = MIMEImage((attach_file).read(), name=os.path.basename(self._file_img))
                self._message.attach(image)
        if self._file_video is not None:
            with open(self._file_video,'rb') as attach_file: # Open the file as binary mode
                video = MIMEApplication((attach_file).read(), name=os.path.basename(self._file_video))
                self._message.attach(video) 
        #Create SMTP session for sending the mail
        session = smtplib.SMTP('smtp.gmail.com', 587) #use gmail with port
        session.starttls() #enable security
        session.login(self._sender_address, self._sender_pass) #login with mail_id and password
        text = self._message.as_string()
        session.sendmail(self._sender_address, self._receiver_address, text)
        session.quit()
        print('ProposalYesListener Notification Sent')

class EventManager:
    def __init__(self, event_filters, listeners):
        self._event_filters = event_filters
        self._listeners = listeners

    def handle_event(self, event):
        print(event)
        for listener_dict in self._listeners:
            listener = listener_dict.get(event['event'], None)
            if listener is not None:
                listener.notify(event['args'])

    def loop(self, poll_interval):
        while True:
            for event_filter in self._event_filters:
                for event in event_filter.get_new_entries():
                    self.handle_event(event)
            time.sleep(poll_interval)



def main():
    listeners = [{'LocationVerifiedEvent': ProposalGeofenceListener(receiver = 'yourmail@mail.com')}, 
    {'ProposalEvent': ProposalYesListener(file_img = "/home/pi/camera/his_proof.jpg", file_video = "/home/pi/camera/common_proof.h264", receiver = 'yourmail@mail.com')}, 
    {'ProposalEvent': ProposalYesListener(file_img = "/home/pi/camera/her_proof.jpg", file_video = "/home/pi/camera/common_proof.h264", receiver = 'yourmail@mail.com')}
    ]

    with open(f"{dir_path}/abi.json",'r') as abi_file:
        abi = abi_file.read()
    ABI = json.loads(abi)
    # HTTPProvider:
    w3 = Web3(Web3.HTTPProvider("https://mainnet.infura.io/v3/<API-KEY>"))
    res = w3.isConnected()
    print(res)
    contract = w3.eth.contract(address=SMART_PROPOSAL_CONTRACT_ADDRESS, abi=ABI)
    contract_owner = contract.functions.getOwner().call()
    print(f"Connection Test: Contract owner is {contract_owner}")
    event_filters = []
    event_filters.append(contract.events.LocationVerifiedEvent.createFilter(fromBlock='latest'))
    event_filters.append(contract.events.ProposalEvent.createFilter(fromBlock='latest')) 
    event_manager = EventManager(event_filters, listeners)
    event_manager_thread = Thread(target=event_manager.loop, args=(5,), daemon=True)
    event_manager_thread.start()
    print("Start listening")
    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()