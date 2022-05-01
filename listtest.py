import time

def lseq_add_event(start, opcode):
    global lseq
    start_time = time.time() + start
    lseq.insert(0, [start_time, opcode])
    lseq.sort()
    
def lseq_add_block(block):
    global lseq
    for x in block:
        lseq_add_event(x[0], x[1])
    lseq.sort()
    
time_basis = time.time()
az = 0
lseq=[[8.0, 'NAV'], [2, "WHILE AZ <", 90], [4.5, 'STOP'], [5.0, "steer"]]
seq_block=[[1, "STEER", az+89], [3, "FWD", 50],[4, "WHILE AZ <", 90]]

lseq.sort()
while (True):
    if (len(lseq) > 0):
        start_time = lseq[0][0]
        if (time.time() >= start_time):
            opcode = lseq[0][1]
            print("time %d, opcode=%s" % (start_time, opcode))
            if (opcode == "WHILE AZ <"):
                lseq_add_block(seq_block)
            if (opcode == "NAV"):
                lseq_add_event(1, "NAV")
            if (len(lseq[0]) > 2):
                parameter = lseq[0][2]
                print(parameter)
            lseq.pop(0)
        lseq.sort()
    else:
        time.sleep(.1)
        
