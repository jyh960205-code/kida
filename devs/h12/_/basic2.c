// H12 standalone 12-DOF joint position controller, UDP port 1234.
// Joint order matches eio.c/h12.py: X330 ID1, ID2, then 10 CAN flexion axes.
// pid/joint take 12 joint angles in radians. stop removes drive; quit exits.
// X330 uses current mode + external PD, just like eio.c + h12.py.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "sockcan.h"
#include "dynamixel_sdk.h"

#define N_JOINT 12
#define N_DRIVER 5
#define N_PULSES_PER_REV 120000
#define DT 0.002
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_CURRENT 102
#define ADDR_PRESENT_POSITION 132

static int send_id[N_DRIVER] = {0x10, 0x20, 0x30, 0x40, 0x50};
static int flx_motor_map[10], encoder_dir[12], motor_dir[12], dxl_off[2];
static int dpos[12], recv_id;
static int fd1 = -1, fd2 = -1, sd = -1;
static int groupwrite_num, groupread_num, dxl_open;
static volatile sig_atomic_t exiting;
static int mode;
static double target[12];

static int fail_step(const char *msg){
    fprintf(stderr, "%s\n", msg);
    fflush(stderr);
    return -1;
}

static int dxl_status(const char *what, int id){
    int result = getLastTxRxResult(fd2, 2.0);
    int error = getLastRxPacketError(fd2, 2.0);
    if (result == COMM_SUCCESS && error == 0) return 0;
    fprintf(stderr, "X330 ID%d %s: %s / %s\n", id, what,
            getTxRxResult(2.0, result), getRxPacketError(2.0, error));
    return -1;
}

static void on_signal(int sig){ (void)sig; exiting = 1; }

static double seconds_between(struct timespec a, struct timespec b){
    return (a.tv_sec-b.tv_sec) + (a.tv_nsec-b.tv_nsec)*1e-9;
}

static void m2q(double *m, double *q){
    double ratio = 0.25;
    q[0] = m[0];
    q[1] = m[1];
    q[2] = m[2];
    q[3] = m[3] + ratio*q[2];
    q[4] = m[4];
    q[5] = m[5] + ratio*q[4];
    q[6] = m[6];
    q[7] = m[7] + ratio*q[6];
    q[8] = m[8];
    q[9] = m[9] + ratio*q[8];
    q[10] = m[10];
    q[11] = m[11] + ratio*q[10];
}

static int write_dynamixel_current(int *mA){
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_addparam_result = False;

    groupSyncWriteClearParam(groupwrite_num);
    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, 1, mA[0], 2);
    if (dxl_addparam_result != True) return fail_step("[ID1] groupSyncWrite addparam failed");
    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, 2, mA[1], 2);
    if (dxl_addparam_result != True) return fail_step("[ID2] groupSyncWrite addparam failed");

    groupSyncWriteTxPacket(groupwrite_num);
    groupSyncWriteClearParam(groupwrite_num);
    if ((dxl_comm_result = getLastTxRxResult(fd2, 2.0)) != COMM_SUCCESS) {
        fprintf(stderr, "Dynamixel write failed: %s\n", getTxRxResult(2.0, dxl_comm_result));
        return -1;
    }
    groupSyncWriteClearParam(groupwrite_num);
    return 0;
}

static int read_dynamixel_position(void){
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_getdata_result = False;

    groupSyncReadTxRxPacket(groupread_num);
    if ((dxl_comm_result = getLastTxRxResult(fd2, 2.0)) != COMM_SUCCESS) {
        fprintf(stderr, "Dynamixel read failed: %s\n", getTxRxResult(2.0, dxl_comm_result));
        return -1;
    }
    if ((dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, 1, ADDR_PRESENT_POSITION, 4)) != True) return fail_step("[ID1] groupSyncRead getdata failed");
    if ((dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, 2, ADDR_PRESENT_POSITION, 4)) != True) return fail_step("[ID2] groupSyncRead getdata failed");

    dpos[0] = groupSyncReadGetData(groupread_num, 1, ADDR_PRESENT_POSITION, 4);
    dpos[1] = groupSyncReadGetData(groupread_num, 2, ADDR_PRESENT_POSITION, 4);
    return 0;
}

static int exchange_can_duty(int *duty_cmd){
    int tmp[10];

    for(int i = 0; i < N_DRIVER; i++) {
        if (can_send_u16s(fd1, send_id[i], duty_cmd+2*i, 2) < 0) return -1;
        int ret = can_recv_u32s(fd1, &recv_id, tmp+2*i, 2);
        if (ret <= 0) {
            fprintf(stderr, "CAN-FD receive failed at frame %d send_id=0x%x ret=%d\n", i, send_id[i], ret);
            fflush(stderr);
            return -1;
        }
    }
    for(int i = 0; i < 10; i++) dpos[i+2] = tmp[flx_motor_map[i]];
    return 0;
}

// Same observation stages as eio.c; basic uses measured time for velocity.
static void update_observation(double *q, double *qd, double *old_q,
                               struct timespec *previous, int update_velocity){
    double m[12];
    for (int i = 0; i < 2; i++)
        m[i] = encoder_dir[i] * M_PI * ((double)dpos[i] - dxl_off[i])/2048.0;
    for (int i = 2; i < N_JOINT; i++)
        m[i] = encoder_dir[i] * 2.0*M_PI * dpos[i]/N_PULSES_PER_REV;
    m2q(m, q);
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = seconds_between(now, *previous);
    for (int i = 0; i < N_JOINT; i++) {
        qd[i] = update_velocity && dt > 0 ? (q[i]-old_q[i])/dt : 0.0;
        old_q[i] = q[i];
    }
    *previous = now;
}

static void usage(const char *prog){
    printf("usage: %s -t 0|1 [-c CAN-index] [-u ttyUSB-index] [-v]\n"
           "  -t 0=left, 1=right (required); -c/-u default to 0\n"
           "  UDP :1234: pid/joint q0 ... q11 (radians), zero, test, stop, quit\n"
           "  q0/q1=X330 ID1/ID2, q2..q11=CAN flexion joints (eio.c order)\n"
           "  zero: 0 0.5 0 0 0 0 0 0 0 0 0 0\n"
           "  test: 0.3 1.0 0.5 0.5 0 0 0 0 0 0 0 0\n"
           "  Starts stopped; stop sends zero current/PWM, quit disables X330 torque.\n", prog);
}

int main(int argc, char **argv){
    int ch = 0, tty = 0, type = -1, verbose = 0, c, rc = 0;
    while ((c = getopt(argc, argv, "c:u:t:vh")) != -1) {
        if (c == 'h') { usage(argv[0]); return 0; }
        if (c == 'v') { verbose = 1; continue; }
        if (c != 'c' && c != 'u' && c != 't') { usage(argv[0]); return 64; }
        char *end;
        errno = 0;
        long value = strtol(optarg, &end, 10);
        if (errno || end == optarg || *end || value < 0 || value > INT_MAX) {
            usage(argv[0]); return 64;
        }
        if (c == 'c') ch = (int)value;
        else if (c == 'u') tty = (int)value;
        else type = (int)value;
    }
    if (optind != argc || (type != 0 && type != 1)) { usage(argv[0]); return 64; }
    // Hand calibration from eio.c.
    {
        //left hand case
        if (type == 0){
            flx_motor_map[0] = 0; flx_motor_map[1] = 1; flx_motor_map[2] = 3; flx_motor_map[3] = 2; flx_motor_map[4] = 5; flx_motor_map[5] = 4; flx_motor_map[6] = 7; flx_motor_map[7] = 6; flx_motor_map[8] = 9; flx_motor_map[9] = 8;
            motor_dir[0] = -1; motor_dir[1] = 1; motor_dir[2] = 1; motor_dir[3] = 1; motor_dir[4] = 1; motor_dir[5] = 1; motor_dir[6] = 1; motor_dir[7] = 1; motor_dir[8] = 1; motor_dir[9] = 1; motor_dir[10] = 1; motor_dir[11] = 1;
            encoder_dir[0] = -1; encoder_dir[1] = 1; encoder_dir[2] = 1; encoder_dir[3] = 1; encoder_dir[4] = 1; encoder_dir[5] = 1; encoder_dir[6] = 1; encoder_dir[7] = 1; encoder_dir[8] = 1; encoder_dir[9] = 1; encoder_dir[10] = 1; encoder_dir[11] = 1;
            dxl_off[0] = 1486;
            dxl_off[1] = 1540; //517;
        }

        //right hand case
        else if(type == 1){
            flx_motor_map[0] = 0; flx_motor_map[1] = 1; flx_motor_map[2] = 3; flx_motor_map[3] = 2; flx_motor_map[4] = 5; flx_motor_map[5] = 4; flx_motor_map[6] = 7; flx_motor_map[7] = 6; flx_motor_map[8] = 9; flx_motor_map[9] = 8;
            motor_dir[0] = 1; motor_dir[1] = -1; motor_dir[2] = 1; motor_dir[3] = 1; motor_dir[4] = 1; motor_dir[5] = 1; motor_dir[6] = 1; motor_dir[7] = 1; motor_dir[8] = 1; motor_dir[9] = 1; motor_dir[10] = 1; motor_dir[11] = 1;
            encoder_dir[0] = 1; encoder_dir[1] = -1; encoder_dir[2] = 1; encoder_dir[3] = 1; encoder_dir[4] = 1; encoder_dir[5] = 1; encoder_dir[6] = 1; encoder_dir[7] = 1; encoder_dir[8] = 1; encoder_dir[9] = 1; encoder_dir[10] = 1; encoder_dir[11] = 1;
            dxl_off[0] = 1486;
            dxl_off[1] = 517;
        }

        else {
            printf("Hand type=%d should be 0(left) or 1(right)\n", type);
            return 64;
        }
    }
    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) { perror("UDP socket"); return 1; }
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(1234);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("UDP bind"); rc = 1; goto shutdown;
    }
    // Initialize CAN and X330.
    {
        fd1 = can_init(ch);
        if (fd1 < 0) { rc = 1; goto shutdown; }
        char name[32];
        snprintf(name, sizeof(name), "/dev/ttyUSB%d", tty);
        fd2 = portHandler(name);
        if (fd2 < 0) { fail_step("Dynamixel portHandler failed"); rc = 1; goto shutdown; }
        packetHandler();
        if (!openPort(fd2)) { fail_step("Dynamixel openPort failed"); rc = 1; goto shutdown; }
        dxl_open = 1;
        if (!setBaudRate(fd2, 1000000)) { fail_step("Dynamixel baudrate failed"); rc = 1; goto shutdown; }
        groupwrite_num = groupSyncWrite(fd2, 2.0, ADDR_GOAL_CURRENT, 2);
        groupread_num = groupSyncRead(fd2, 2.0, ADDR_PRESENT_POSITION, 4);
        for (int id = 1; id <= 2; id++) {
            write1ByteTxRx(fd2, 2.0, id, ADDR_TORQUE_ENABLE, 0);
            if (dxl_status("disable torque", id) < 0) { rc = 1; goto shutdown; }
            // External position PD requires current control mode (0).
            int operating_mode = read1ByteTxRx(fd2, 2.0, id, ADDR_OPERATING_MODE);
            if (dxl_status("read operating mode", id) < 0) { rc = 1; goto shutdown; }
            if (operating_mode != 0) {
                write1ByteTxRx(fd2, 2.0, id, ADDR_OPERATING_MODE, 0);
                if (dxl_status("set current mode", id) < 0) { rc = 1; goto shutdown; }
            }
            write2ByteTxRx(fd2, 2.0, id, ADDR_GOAL_CURRENT, 0);
            if (dxl_status("clear current", id) < 0) { rc = 1; goto shutdown; }
            if (!groupSyncReadAddParam(groupread_num, id))
                { fail_step("Dynamixel sync read addparam failed"); rc = 1; goto shutdown; }
        }
        for (int id = 1; id <= 2; id++) {
            write1ByteTxRx(fd2, 2.0, id, ADDR_TORQUE_ENABLE, 1);
            if (dxl_status("enable torque", id) < 0) { rc = 1; goto shutdown; }
        }
    }
    printf("H12 %s: can%d, /dev/ttyUSB%d, UDP :1234; stopped\n",
           type == 0 ? "left" : "right", ch, tty);

    double q[12], old_q[12], qd[12] = {0}, tau[12] = {0};
    int current[2], duty[10];
    struct timespec previous = {0}, next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    long count = 0;
    while (!exiting) {
        // Bounded work per control tick; all targets update on this thread.
        for (int i = 0; i < 32 && !exiting; i++) {
            char buf[4096];
            ssize_t n = recv(sd, buf, sizeof(buf)-1, MSG_DONTWAIT | MSG_TRUNC);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) break;
                perror("UDP recv");
                { rc = 1; goto shutdown; }
            }
            if ((size_t)n >= sizeof(buf) || memchr(buf, '\0', (size_t)n)) {
                fprintf(stderr, "Invalid UDP command length/content\n");
                continue;
            }
            buf[n] = '\0';
            {
                char *next_word;
                char *word = strtok_r(buf, " \t\r\n", &next_word);
                if (!word) goto command_done;
                double candidate[12] = {0};
                int new_mode = 1;
                if (!strcmp(word, "pid") || !strcmp(word, "joint")) {
                    for (int i = 0; i < N_JOINT; i++) {
                        char *value = strtok_r(NULL, " \t\r\n", &next_word), *end;
                        if (!value) goto invalid;
                        errno = 0;
                        candidate[i] = strtod(value, &end);
                        if (end == value || *end || errno || !isfinite(candidate[i])) goto invalid;
                    }
                } else if (!strcmp(word, "stop")) new_mode = 0;
                else if (!strcmp(word, "quit")) new_mode = -1;
                else if (!strcmp(word, "zero")) {
                    // Match h12.py's zero pose, including the second X330 joint.
                    candidate[1] = 0.5;
                } else if (!strcmp(word, "test")) {
                    candidate[0] = 0.3; candidate[1] = 1.0;
                    candidate[2] = 0.5; candidate[3] = 0.5;
                } else goto invalid;
                if (strtok_r(NULL, " \t\r\n", &next_word)) goto invalid;
                if (new_mode < 0) exiting = 1;
                else {
                    memcpy(target, candidate, sizeof(target));
                    mode = new_mode;
                }
                goto command_done;
                invalid:
                fprintf(stderr, "Invalid command: pid/joint needs exactly 12 finite radians; stop/zero/test/quit take no arguments\n");
                goto command_done;
                command_done:
                    ;
            }
            // Apply stop before processing any subsequently queued movement.
            if (mode == 0) break;
        }
        if (exiting) break;
        // The first transaction is zero drive to obtain initial joint positions.
        if (count) {
            for (int i = 0; i < N_JOINT; i++) {
                // X330 gains from h12.py; retain basic.c's CAN gains.
                double kp = i < 2 ? 1.0 : 0.3;
                double kd = i < 2 ? 0.02 : 0.005;
                tau[i] = mode ? kp*(target[i] - q[i]) - kd*qd[i] : 0.0;
            }
        }
        // PD output -> X330 current and CAN PWM.
        {
            int tmp[10];
            for (int i = 0; i < 2; i++) {
                // Same raw Goal Current scaling and +/-600 cap as eio.c.
                double value = motor_dir[i] * 1000.0*tau[i];
                current[i] = (int)fmax(-600.0, fmin(600.0, value));
            }
            for (int i = 0; i < 10; i++) {
                double v = fmax(-1.0, fmin(1.0, motor_dir[i+2]*tau[i+2]));
                tmp[i] = (int)(0x8000*v) + 0x8000;
                if (tmp[i] > 65535) tmp[i] = 65535;
            }
            for (int i = 0; i < 10; i++) duty[i] = tmp[flx_motor_map[i]];
        }
        if (write_dynamixel_current(current) < 0) { rc = 1; goto shutdown; }
        if (read_dynamixel_position() < 0) { rc = 1; goto shutdown; }
        if (exchange_can_duty(duty) < 0) { rc = 1; goto shutdown; }
        update_observation(q, qd, old_q, &previous, count > 0);
        struct timespec now = previous;
        if (verbose && count % 50 == 0) {
            printf("[%ld mode:%d]", count, mode);
            for (int i = 0; i < N_JOINT; i++) printf(" %6.3f", q[i]);
            putchar('\n');
        }
        count++;
        next.tv_nsec += (long)(DT*1e9);
        if (next.tv_nsec >= 1000000000L) { next.tv_sec++; next.tv_nsec -= 1000000000L; }
        // Skip missed deadlines; velocity uses measured time, not assumed DT.
        if (seconds_between(now, next) > 0) next = now;
        int error;
        do { error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL); }
        while (error == EINTR && !exiting);
        if (error && error != EINTR) { rc = 1; break; }
    }
shutdown:
    {
        // Stop X330 first; a missing CAN board must not delay torque disable.
        if (dxl_open) {
            for (int id = 1; id <= 2; id++) {
                write2ByteTxRx(fd2, 2.0, id, ADDR_GOAL_CURRENT, 0);
                dxl_status("shutdown current", id);
                write1ByteTxRx(fd2, 2.0, id, ADDR_TORQUE_ENABLE, 0);
                dxl_status("shutdown torque", id);
            }
            closePort(fd2);
        }
        if (fd1 >= 0) {
            int zero[2] = {0x8000, 0x8000};
            for (int i = 0; i < N_DRIVER; i++) can_send_u16s(fd1, send_id[i], zero, 2);
            close(fd1);
        }
        if (sd >= 0) close(sd);
    }
    return rc;
}
