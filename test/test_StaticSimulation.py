#!/bin/python3

import os
import time
import requests
import unittest

API_URL = "http://localhost:9000"

class TestSingleNode(unittest.TestCase):
    def testPostStaticSimulation(self):
        
        self.assertFalse(os.path.exists("data/out/edgedata-static.xml"))
        self.assertFalse(os.path.exists("data/out/routes-static.xml"))
        r = requests.post(f"{API_URL}/static/simulation/porto-full-test", json={
            "netPath": "porto/porto-armis.net.xml",
            "tazPath": "porto/porto-armis.taz.xml",
            "demandPath": "od/matrix.9.0.10.0.2.fma",
            "outEdgesPath": "out/edgedata-static.xml",
            "outRoutesPath": "out/routes-static.xml"
        })
        print("Status code: ", r.status_code)
        print("Text: ", r.text)
        self.assertEqual(r.status_code, 200)
        
        time.sleep(1)
        
        r = requests.get(f"{API_URL}/static/simulation/porto-full-test/join")
        print("Status code: ", r.status_code)
        print("Text: ", r.text)
        self.assertEqual(r.status_code, 200)
        
        self.assertTrue(os.path.exists("data/out/edgedata-static.xml"))
        self.assertTrue(os.path.exists("data/out/routes-static.xml"))

if __name__ == '__main__':
    unittest.main()
