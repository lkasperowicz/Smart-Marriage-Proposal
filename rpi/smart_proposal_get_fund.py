import os
from time import sleep, time
import shutil
import json
from datetime import datetime
from enum import Enum
from random import randint
from threading import Thread, Lock
import base64
from web3 import Web3
from eth_utils import encode_hex
from eth_abi import encode_abi

dir_path = os.path.dirname(os.path.realpath(__file__))

SMART_PROPOSAL_CONTRACT_ADDRESS ='0xe4e7Ed4f714A8cbDA6Da05910484CFA796f992bb'
SMART_PROPOSAL_BOX_ADDRESS = '0x7C7B3A6C355da24fDF567d400A498a7e51315109'
SMART_PROPOSAL_CONTRACT_TRANSACTION_GAS_LIMIT = 70000
SMART_PROPOSAL_CONTRACT_TRANSACTION_GAS_FEE_LIMIT = 300
SMART_PROPOSAL_CONTRACT_TRANSACTION_PRIORITY_FEE_LIMIT = 2
PRIVATE_KEY = ''

with open(f"{dir_path}/secure/smart_proposal_box.key") as keyfile:
    PRIVATE_KEY = str(keyfile.read())

def SendSmartContractTransaction(transaction):
    retransmission = 5
    while retransmission > 0:
        try:
            transaction.update({'maxFeePerGas': w3.toWei(SMART_PROPOSAL_CONTRACT_TRANSACTION_GAS_FEE_LIMIT, 'gwei')})
            transaction.update({'maxPriorityFeePerGas': w3.toWei(SMART_PROPOSAL_CONTRACT_TRANSACTION_PRIORITY_FEE_LIMIT, 'gwei')})
            transaction.update({'nonce' : w3.eth.get_transaction_count(SMART_PROPOSAL_BOX_ADDRESS)})
            signed_transaction = w3.eth.account.sign_transaction(transaction, PRIVATE_KEY)
            print(transaction)
            transaction_hash = w3.eth.send_raw_transaction(signed_transaction.rawTransaction)
            print(f"Transaction hash: {encode_hex(transaction_hash)}")
            retransmission = 0
        except:
            retransmission = retransmission - 1

def SendGetProposalFundTransaction(her_proof, his_proof):
    print("SendGetProposalFundTransaction")
    contract = w3.eth.contract(address=SMART_PROPOSAL_CONTRACT_ADDRESS, abi=ABI)
    transaction = contract.functions.getProposalFund(her_proof, his_proof).buildTransaction({'value': w3.toWei(0.00035, 'ether')})
    SendSmartContractTransaction(transaction)

with open(f"{dir_path}/abi.json",'r') as abi_file:
    abi = abi_file.read()
ABI = json.loads(abi)
# HTTPProvider:
w3 = Web3(Web3.HTTPProvider("https://mainnet.infura.io/v3/<API-KEY>"))
res = w3.isConnected()
print("Web3 is connected")
w3.eth.defaultAccount = SMART_PROPOSAL_BOX_ADDRESS

#main loop
with open("/home/pi/camera/her_proof.jpg", "rb") as her_proof:
    her_proof_source_string = base64.b64encode(her_proof.read())
    # only one hash
    her_proof_bytes = Web3.keccak(her_proof_source_string)
with open("/home/pi/camera/his_proof.jpg", "rb") as his_proof:
    his_proof_source_string = base64.b64encode(his_proof.read())
    # only one hash
    his_proof_bytes = Web3.keccak(his_proof_source_string)

SendGetProposalFundTransaction(her_proof_bytes, his_proof_bytes)

