#!/bin/python3

import os
import requests
import unittest

API_URL = "http://localhost:8000"

class TestSingleNode(unittest.TestCase):
    def testPostStaticSimulation(self):
        
        networkId = "bpr1"
        demandId = "demand1"
        
        self.assertFalse(os.path.exists("data/out/edgedata-static.xml"))

        r = requests.put(f"{API_URL}/static/network/{networkId}", json={
            "netPath": "network/net.net.xml",
            "tazPath": "network/taz.xml",
            "model": "BPR"
        })
        self.assertEqual(r.status_code, 200)
        
        r = requests.put(f"{API_URL}/static/demand/{demandId}", json={
            "networkId": networkId,
            "srcPath": "od/matrix.8.0.9.0.1.fma"
        })
        self.assertEqual(r.status_code, 200)
        
        
        r = requests.post(f"{API_URL}/static/simulation", json={
            "networkId": networkId,
            "demandId": demandId,
            "dstPath": "out/edgedata-static.xml"
        })
        self.assertEqual(r.status_code, 200)
        self.assertTrue(os.path.exists("data/out/edgedata-static.xml"))

if __name__ == '__main__':
    unittest.main()
