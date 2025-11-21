#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <string.h>
#include <pthread.h>
#include "config.h"
#include "control.h"
#include "crsf_protocol.h"

static int initialized = 0;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

Config parse_yaml(const std::string &filepath)
{
    try {
        YAML::Node config_root = YAML::LoadFile(filepath);
        Config cfg;

        cfg.vrx_switch.channel = config_root["vrx_switch"]["channel"].as<int>();
        cfg.vrx_switch.pwm = config_root["vrx_switch"]["pwm"].as<std::vector<int>>();

        cfg.tx_switch.channel = config_root["tx_switch"]["channel"].as<int>();
        cfg.tx_switch.pwm = config_root["tx_switch"]["pwm"].as<std::vector<int>>();

        for (const auto &node : config_root["vrx_table"]) {
            cfg.vrx_table.push_back(node.as<VrxBand>());
        }

        for (const auto &node : config_root["vtx_table_58"]) {
            cfg.vtx_table_58.push_back(node.as<VtxBand>());
        }

        for (const auto &node : config_root["power_table"]) {
            cfg.power_table.push_back(node.as<PowerSetting>());
        }
        cfg.app_config.ctl_port = config_root["application"]["control_port"].as<int>();
        cfg.app_config.crsf_port = config_root["application"]["crsf_port"].as<int>();
        cfg.app_config.stream_port = config_root["application"]["stream_port"].as<int>();
        cfg.app_config.antenna_ip = config_root["application"]["antenna_ip"].as<std::string>();
        cfg.app_config.uart_dev = config_root["application"]["crsf_device"].as<std::string>();
        if (config_root["application"]["baudrate"].IsDefined())
            cfg.app_config.baudrate = config_root["application"]["baudrate"].as<int>();
        else
            cfg.app_config.baudrate = 115200;
        printf("%s: baudrate %d\n", __func__, cfg.app_config.baudrate);
        return cfg;
    }
    catch (const YAML::BadFile &e) {
        std::cerr << "Can't open file " << filepath << std::endl;
        throw;
    }
    catch (const YAML::Exception &e) {
        std::cerr << "Can't parse file " << e.what() << std::endl;
        throw;
    }
}
static Config config;

extern "C" int load_config(const char *conf_name)
{
    printf("Loading file %s\n", conf_name);
    try {
        pthread_mutex_lock(&mutex);
        config = parse_yaml(conf_name);
        initialized = 1;
    }
    catch (const std::exception &e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        pthread_mutex_unlock(&mutex);
        return -1;
    }
    pthread_mutex_unlock(&mutex);
    return 0;
}

static uint16_t get_channel_us(crsf_channels_t *crsf, int chan_num)
{
    uint16_t ticks;

    if (!crsf)
        return 0;

    switch (chan_num) {
    case 0:
        ticks = crsf->ch0;
        break;
    case 1:
        ticks = crsf->ch1;
        break;
    case 2:
        ticks = crsf->ch2;
        break;
    case 3:
        ticks = crsf->ch3;
        break;
    case 4:
        ticks = crsf->ch4;
        break;
    case 5:
        ticks = crsf->ch5;
        break;
    case 6:
        ticks = crsf->ch6;
        break;
    case 7:
        ticks = crsf->ch7;
        break;
    case 8:
        ticks = crsf->ch8;
        break;
    case 9:
        ticks = crsf->ch9;
        break;
    case 10:
        ticks = crsf->ch10;
        break;
    case 11:
        ticks = crsf->ch11;
        break;
    case 12:
        ticks = crsf->ch12;
        break;
    case 13:
        ticks = crsf->ch13;
        break;
    case 14:
        ticks = crsf->ch14;
        break;
    case 15:
        ticks = crsf->ch15;
        break;
    default:
        return 0;
    }
    return TICKS_TO_US(ticks);
}

int get_chan_info(crsf_channels_t *crsf, struct channel_data *data, unsigned int *size)
{
    uint16_t usval;
    int cnt = 0;
    int active = -1;
    if (!initialized) {
        *size = 0;
        return 0;
    }
    pthread_mutex_lock(&mutex);
    usval = get_channel_us(crsf, config.vrx_switch.channel - 1);
    if (!usval)
        return -1;
    for (unsigned int i = 0; i < config.vrx_switch.pwm.size(); i++)
        if (usval < config.vrx_switch.pwm[i]) {
            active = i;
            break;
        }
    data->tx_selected = 0;
    usval = get_channel_us(crsf, config.tx_switch.channel - 1);
    for (unsigned int i = 0; i < config.tx_switch.pwm.size(); i++)
        if (usval < config.tx_switch.pwm[i]) {
            data->tx_selected = i;
            break;
        }
    for (unsigned int i = 0; i < config.vrx_table.size(); i++, cnt++) {
        if (i >= *size)
            break;
        usval = get_channel_us(crsf, config.vrx_table[i].channel - 1);
        data[cnt].selected = 0;
        if (config.vrx_table[i].id == active) {
            data[cnt].selected = 1;
        }
        for (unsigned int j = 0; j < config.vrx_table[i].freqs.size(); j++) {
            if (usval <= config.vrx_table[i].pwm[j]) {
                data[cnt].freq = config.vrx_table[i].freqs[j];
                if (config.vrx_table[i].bands.size() > j) {
                    snprintf(data[cnt].band_name, sizeof(data[cnt].band_name), "%s:%s",
                            config.vrx_table[i].name.c_str(), config.vrx_table[i].bands[j].c_str());
                } else {
                    snprintf(data[cnt].band_name, sizeof(data[cnt].band_name), "%s:%d",
                             config.vrx_table[i].name.c_str(), j+1);
                }
                break;
            }
        }
    }
    *size = cnt;
    pthread_mutex_unlock(&mutex);
    return *size;
}

int get_table_info(struct vrx_table *table, unsigned int *size)
{
    if (!initialized) {
        *size = 0;
        return 0;
    }
    pthread_mutex_lock(&mutex);
    unsigned int cnt = 0;
    for (unsigned int i = 0; i < config.vrx_table.size(); i++, cnt++) {
        if (i >= *size)
            break;
        memset(&table[i].table, 0, sizeof(table[i].table));
        table[i].band = static_cast<uint8_t>(config.vrx_table[i].baseband);
        for (unsigned int j = 0; j < config.vrx_table[i].freqs.size() && j < 16; j++) {
            table[i].table[j] = static_cast<uint16_t>(config.vrx_table[i].freqs[j]);
        }
    }
    *size = cnt;
    pthread_mutex_unlock(&mutex);
    return *size;
}

int get_app_config(app_config_t *cfg)
{
    if (!cfg)
        return -1;
    cfg->ctl_port = config.app_config.ctl_port;
    cfg->crsf_port = config.app_config.crsf_port;
    cfg->stream_port = config.app_config.stream_port;
    if (cfg->antenna_ip)
        free(cfg->antenna_ip);
    cfg->antenna_ip = strdup(config.app_config.antenna_ip.c_str());
    if (cfg->uart_dev)
        free(cfg->uart_dev);
    cfg->uart_dev = strdup(config.app_config.uart_dev.c_str());
    cfg->baudrate = config.app_config.baudrate;
    return 0;
}