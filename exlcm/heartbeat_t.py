"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class heartbeat_t(object):

    __slots__ = ["timestamp", "ID"]

    __typenames__ = ["int64_t", "int8_t"]

    __dimensions__ = [None, None]

    def __init__(self):
        self.timestamp = 0
        """ LCM Type: int64_t """
        self.ID = 0
        """ LCM Type: int8_t """

    def encode(self):
        buf = BytesIO()
        buf.write(heartbeat_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qb", self.timestamp, self.ID))

    @staticmethod
    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != heartbeat_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return heartbeat_t._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = heartbeat_t()
        self.timestamp, self.ID = struct.unpack(">qb", buf.read(9))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if heartbeat_t in parents: return 0
        tmphash = (0xedba2cef5dac009a) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if heartbeat_t._packed_fingerprint is None:
            heartbeat_t._packed_fingerprint = struct.pack(">Q", heartbeat_t._get_hash_recursive([]))
        return heartbeat_t._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", heartbeat_t._get_packed_fingerprint())[0]

