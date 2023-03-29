#!/bin/python3

import os
import requests
import unittest

API_URL = "http://localhost:9000"

class TestSingleNode(unittest.TestCase):
    def testPostStaticSimulation(self):
        
        self.assertFalse(os.path.exists("data/out/edgedata-static.xml"))
        r = requests.post(f"{API_URL}/static/simulation", json={
            "netPath": "network/net.net.xml",
            "tazPath": "network/taz.xml",
            "demandPath": "od/matrix.9.0.10.0.2.fma",
            "dstPath": "out/edgedata-static.xml"
        })
        self.assertEqual(r.status_code, 200)
        self.assertTrue(os.path.exists("data/out/edgedata-static.xml"))

if __name__ == '__main__':
    unittest.main()
