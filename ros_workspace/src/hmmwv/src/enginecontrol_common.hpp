#pragma once

const uint8_t   CMD_STOP    = 0;
const uint8_t   CMD_DRIVE   = 1;

enum Direction {
    CW, CCW
};

enum EngineId {
    ENGINE_LEFT, ENGINE_RIGHT
};

struct cmd {
    uint8_t cmd_id:4,
            engine_id:3,
            direction:1;
    uint8_t data;
};

void marshal(const struct cmd *cmd, uint8_t *buf) {
    buf[0] = (cmd->cmd_id<<4) + (cmd->engine_id<<1) + cmd->direction;
    buf[1] = cmd->data;
}

void unmarshal(struct cmd *cmd, const uint8_t *buf) {
    cmd->cmd_id = (buf[0]>>4) & 0x0f;
    cmd->engine_id = (buf[0]>>1) & 0x07;
    cmd->direction = buf[0] & 0x01;
    cmd->data = buf[1];
}
