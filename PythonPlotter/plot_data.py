import argparse
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
import serial
import threading
from queue import Queue, Empty
import time
import serial.tools.list_ports

# Serial port configuration
BAUD_RATE = 115200  # Match the Arduino's Serial.begin(115200)

from serial_parser import parse_message, SIData, SDData, parse_data, parse_all_messages
# SIPlot is no longer used directly; SI plots are embedded inside SDPlot now when include_si=True
from sd_plot import SDPlot


# SDData dataclass has been moved into serial_parser


# parse_message & parse_data have moved into serial_parser


def find_ports():
    return [p.device for p in serial.tools.list_ports.comports() if any(tok in p.device for tok in ('ACM', 'COM', 'ttyUSB', 'ttyACM', 'USB'))]


def main():
    parser = argparse.ArgumentParser(description='Plot data from Arduino serial output')
    parser.add_argument('--port', '-p', help='Serial port to use (overrides auto-detection)')
    parser.add_argument('--baud', '-b', type=int, default=BAUD_RATE, help='Baud rate')
    parser.add_argument('--debug', action='store_true', help='Print raw unparsed serial lines')
    parser.add_argument('--sd-decimate', type=int, default=10, help='Decimation factor for SD sensor plotting')
    parser.add_argument('--sd-live', action='store_true', help='Update SD plots live for every SD sample (overrides --sd-decimate)')
    parser.add_argument('--si-live', action='store_true', help='Update SI plots live (no decimation / update every sample)')
    parser.add_argument('--si-update-every', type=int, default=10, help='Update SI plot every N messages (default 10)')
    parser.add_argument('--si-interval', type=float, default=None, help='Time in seconds between SI plot updates (overrides --si-update-every if set)')
    parser.add_argument('--sd-interval', type=float, default=None, help='Time in seconds between SD plot updates (overrides sd-decimate if set)')
    parser.add_argument('--sd-scroll', type=int, default=20, help='Autoscroll to show the last N samples for SD (0 to disable)')
    parser.add_argument('--no-thread', action='store_true', help='Disable background serial reader thread (default: enabled)')
    args = parser.parse_args()

    # Find available ports
    if args.port:
        ports = [args.port]
    else:
        ports = find_ports()
        if not ports:
            print('No serial ports found; check your connection.')
            return

    ser = None
    for port in ports:
        try:
            ser = serial.Serial(port, args.baud, timeout=1)
            time.sleep(2)
            print(f"Opened serial {port} @ {args.baud}")
            break
        except serial.SerialException:
            continue

    if ser is None:
        print('Failed to open any serial port')
        return

    # initialize plot
    plt.ion()
    # amount of history stored for the plots. Increase SD plots to 200 entries
    max_history = 200
    si_update = 1 if args.si_live else max(1, args.si_update_every)
    # Combine SI and SD plots into one figure by enabling include_si
    sd_plot = SDPlot(sd_decimate=args.sd_decimate, max_history=max_history, display_interval=args.sd_interval, scroll_size=args.sd_scroll, include_si=True, si_update_every=si_update, si_display_interval=args.si_interval)

    print('Listening to serial...')
    # use new SDPlot and SIPlot helper modules to isolate concerns
    # sd_plot already initialized above with include_si=True
    # If live SD updates requested, configure SDPlot for smooth blit updates
    if args.sd_live:
        try:
            sd_plot.enable_live()
        except Exception:
            # if enable_live fails just fall back to normal updates
            if args.debug:
                print('sd_plot.enable_live() failed; falling back to non-blit mode')
    update_counter = 0
    sd_count = 0

    # Optionally run a background reader thread so GUI redraws don't block
    # serial reads. The reader pushes parsed (id, data) tuples into a queue
    # which the main thread drains and uses to update Matplotlib safely.
    q: Queue | None = None
    stop_event = threading.Event()

    def reader_loop(ser, q, stop_event, debug):
        """Background loop which reads serial lines, parses them and
        puts (msg_id, data) tuples in the queue. Runs until stop_event is set.
        """
        while not stop_event.is_set():
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
            except OSError as e:
                if debug:
                    print(f"Serial read error: {e}, retrying...")
                time.sleep(1)
                continue

            if not line:
                # small sleep to avoid busy loop
                time.sleep(0.001)
                continue

            if debug:
                print('RAW:', line)

            msgs = parse_all_messages(line)
            # normalize to array
            parsed_msgs = msgs if isinstance(msgs, list) else [msgs]
            ids = [m for m, _ in parsed_msgs if m is not None]
            if debug and ids:
                print('PARSED:', ids)

            if not parsed_msgs or all(m is None for m, _ in parsed_msgs):
                if debug:
                    print('UNPARSED:', line)
                continue

            for msg in parsed_msgs:
                try:
                    q.put(msg, timeout=0.02)
                except Exception:
                    # drop when queue is full
                    if debug:
                        print('reader_loop: queue full, dropping message')

    if not args.no_thread:
        q = Queue(maxsize=2000)
        reader = threading.Thread(target=reader_loop, args=(ser, q, stop_event, args.debug), daemon=True)
        reader.start()

    # Main loop: if threaded reader is enabled, drain the queue; otherwise
    # do serial reads inline (backwards compatible and handy for debugging).
    try:
        if args.no_thread:
            # Inline (non-threaded) read loop
            while True:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                except OSError as e:
                    print(f"Serial read error: {e}, retrying...")
                    time.sleep(1)
                    continue

                if not line:
                    time.sleep(0.01)
                    continue

                if args.debug:
                    print('RAW:', line)

                msgs = parse_all_messages(line)
                parsed_msgs = msgs if isinstance(msgs, list) else [msgs]
                if args.debug:
                    ids = [m for m, _ in parsed_msgs]
                    print('PARSED:', ids)

                if not parsed_msgs or all(m is None for m, _ in parsed_msgs):
                    if args.debug:
                        print('UNPARSED:', line)
                    continue

                for msg_id, data in parsed_msgs:
                    if msg_id == 'SD':
                        if data is not None:
                            sd_plot.update(data, live=args.sd_live)
                            if args.debug:
                                print('SD parsed and passed to SDPlot')
                    if msg_id == 'SI' or msg_id == 'RAW_CSV':
                        if data is None:
                            if args.debug:
                                print('SI token seen but data parse failed')
                            continue
                        sd_plot.update_si(data)
                        if args.debug:
                            print('SI parsed and included in SD window')
                continue
        else:
            # Threaded read: drain a queue of parsed messages and update plots
            msg_counter = 0
            last_report = time.time()
            while True:
                try:
                    msg_id, data = q.get(timeout=0.05)
                except Empty:
                    # keep GUI alive and responsive
                    plt.pause(0.01)
                    continue

                msg_counter += 1
                if msg_id == 'SD':
                    if data is not None:
                        sd_plot.update(data, live=args.sd_live)
                        if args.debug:
                            print('SD parsed and passed to SDPlot')
                if msg_id == 'SI' or msg_id == 'RAW_CSV':
                    if data is None:
                        if args.debug:
                            print('SI token seen but data parse failed')
                        continue
                    sd_plot.update_si(data)
                    if args.debug:
                        print('SI parsed and included in SD window')

                # periodic throughput print
                now = time.time()
                if now - last_report >= 1.0:
                    if args.debug:
                        print(f"Throughput: {msg_counter} messages/sec, queue_size={q.qsize()}")
                    msg_counter = 0
                    last_report = now

    except KeyboardInterrupt:
        print('Stopping')
    finally:
        # signal background reader to stop and join thread if used
        try:
            stop_event.set()
        except Exception:
            pass
        try:
            if not args.no_thread:
                reader.join(timeout=1)
        except Exception:
            pass
        ser.close()
        plt.close()


if __name__ == '__main__':
    main()
