#!/usr/bin/env python
import numpy as np
import serial
import struct
import os
import re

DEFAULT_DEVICE='/dev/cu.usbmodem401'

class CentSDR():
    def __init__(self, dev = None):
        self.dev = dev or os.getenv('CENTSDR_DEVICE') or DEFAULT_DEVICE
        self.serial = None
        
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        
    def open(self):
        if self.serial is None:
            self.serial = serial.Serial(self.dev)

    def close(self):
        if self.serial:
            self.serial.close()
        self.serial = None

    def send_command(self, cmd):
        self.open()
        self.serial.write(cmd)
        self.serial.readline() # discard empty line

    def set_frequency(self, freq):
        self.send_command("freq %d\r" % freq)

    def set_tune(self, freq):
        self.send_command("tune %d\r" % freq)

    def set_gain(self, gain):
        self.send_command("gain %d\r" % gain)
            
    def set_volume(self, gain):
        self.send_command("volume %d\r" % gain)
        
    def set_fs(self, fs):
        self.send_command("fs %d\r" % fs)
            
    def set_mode(self, mode):
        self.send_command("mode %s\r" % mode)

    def set_channel(self, chan):
        self.send_command("channel %d\r" % chan)
        
    def set_agc(self, agc):
        self.send_command("agc %s\r" % agc)

    def fetch_data(self):
        result = ''
        line = ''
        while True:
            c = self.serial.read()
            if c == chr(13):
                next # ignore CR
            line += c
            if c == chr(10):
                result += line
                line = ''
                next
            if line.endswith('ch>'):
                # stop on prompt
                break
        return result

    def fetch_array(self, sel):
        def hex16(h):
            return struct.unpack('>h', h.decode('hex'))[0]
        self.send_command("data %d\r" % sel)
        data = self.fetch_data()
        x = []
        for line in data.split('\n'):
            if line:
                x.extend([hex16(d) for d in line.strip().split(' ')])
        return np.array(x)

    def data(self, sel = 0):
        self.send_command("data %d\r" % sel)
        data = self.fetch_data()
        x = []
        for line in data.split('\n'):
            if line:
                d = line.strip().split(' ')
                x.append(float(d[0])+float(d[1])*1.j)
        return np.array(x)
    
    def read_power(self):
        self.send_command("power\r")
        resp = self.fetch_data()
        m = re.match(r"power: ([\d.-]+)dBm", resp)
        return float(m.group(1))

    def read_status(self, arg = ''):
        self.send_command("show %s\r" % arg)
        return self.fetch_data()

    def flush_data(self):
        self.send_command("\r")
        self.fetch_data()

def run_as_command():
    from optparse import OptionParser
    parser = OptionParser(usage="%prog [options]")
    parser.add_option("-d", "--dev", dest="device",
                      help="device node (default from env var CENTSDR_DEVICE)", metavar="DEV")
    parser.add_option("-F", "--freqeucy", type="int", dest="freq",
                      help="set tuning frequency", metavar="FREQ")
    parser.add_option("-G", "--gain", type="int", dest="gain",
                      help="gain (0-95)", metavar="GAIN")
    parser.add_option("-V", "--volume", type="string", dest="volume",
                      help="set volume", metavar="VOLUME")
    parser.add_option("-A", "--agc", type="string", dest="agc",
                      help="set agc", metavar="AGC")
    parser.add_option("-M", "--mode", type="string", dest="mode",
                      help="set modulation", metavar="MODE")
    parser.add_option("-C", "--channel", type="string", dest="channel",
                      help="set channel", metavar="MODE")
    parser.add_option("-P", "--power", action="store_true", dest="power",
                      help="show power")
    parser.add_option("-s", "--show", action="store_true", dest="show",
                      help="show current setting")
    parser.add_option("-S", "--show-arg", type='string', dest="showarg", default='',
                      help="show specific setting")
    parser.add_option("-p", "--plot", dest="buffer",
                      help="plot buffer", metavar="BUFFER")
    parser.add_option("-l", "--loop", action="store_true", dest="loop", default=False,
                      help="loop continuously")
    (opt, args) = parser.parse_args()
    sdr = CentSDR(opt.device)
    if opt.freq:
        sdr.set_tune(opt.freq)
    if opt.gain:
        sdr.set_gain(opt.gain)
    if opt.mode:
        sdr.set_mode(opt.mode)
    if opt.volume:
        sdr.set_volume(int(opt.volume))
    if opt.channel:
        sdr.set_channel(int(opt.channel))
    if opt.agc:
        sdr.set_agc(opt.agc)
    if opt.show:
        print sdr.read_status(opt.showarg)
    if opt.buffer:
        import pylab as pl
        sdr.flush_data()
        opt.buffer = int(opt.buffer)
        samp = sdr.fetch_array(opt.buffer)
        if opt.buffer == 0:
            samp = np.array(samp[0::2]) + np.array(samp[1::2])*1j
        x = range(len(samp))
        if opt.loop:
            from signal import signal, SIGINT
            def sigint_handler(signum, frame):
                opt.loop = False
            signal(SIGINT, sigint_handler)
            pl.ion()
        g1 = pl.plot(x, np.real(samp))
        g2 = pl.plot(x, np.imag(samp))
        pl.ylim(-10000,10000)
        pl.xlim(0, len(samp))
        while opt.loop:
            samp = sdr.fetch_array(opt.buffer)
            if opt.buffer == 0:
                samp = np.array(samp[0::2]) + np.array(samp[1::2])*1j
            x = range(len(samp))
            g1[0].set_data(x, np.real(samp))
            g2[0].set_data(x, np.imag(samp))
            pl.draw()
            pl.pause(0.01) 
        pl.show()
        exit(0)
    if opt.power:
        print sdr.read_power()
        exit(0)
        
if __name__ == '__main__':
    run_as_command()
