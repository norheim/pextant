import telnetlib, time, socket


print ("Starting Client...")
host    = '172.20.10.1'
timeout = 120

print ("Connecting...")
try:
    session = telnetlib.Telnet(host, 11123, timeout)
except socket.timeout:
    print ("socket timeout")
else:
    print("Sending Commands...")
    session.write("command".encode('ascii') + b"\r")
    print("Reading...")
    output = session.read_until(b"/r/n/r/n#>", timeout )
    session.close()
    print(output)
    print("Done")