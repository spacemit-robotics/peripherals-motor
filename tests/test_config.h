#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

struct test_config {
    char port_or_can[64];
    int baudrate;
    int ids[16];
    int num_ids;
    int nums;
};

static inline int parse_yaml_config(const char *filename, const char *driver, struct test_config *cfg) {
    cfg->num_ids = 0;
    cfg->baudrate = -1;
    cfg->nums = -1;
    cfg->port_or_can[0] = '\0';
    FILE *f = fopen(filename, "r");
    if (!f) return -1;
    char line[256];
    int in_driver = 0;
    while (fgets(line, sizeof(line), f)) {
        line[strcspn(line, "\r\n")] = 0;
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\0') continue;
        if (strncmp(p, "- name:", 7) == 0) {
            char name[64] = {0};
            sscanf(p + 7, " %63s", name);
            if (strcmp(name, driver) == 0) {
                in_driver = 1;
            } else {
                in_driver = 0;
            }
        } else if (in_driver) {
            if (strncmp(p, "id:", 3) == 0) {
                char *id_str = p + 3;
                while (*id_str == ' ' || *id_str == '\t') id_str++;
                if (*id_str == '[') {
                    id_str++;
                    while (*id_str && *id_str != ']') {
                        int id;
                        if (sscanf(id_str, "%d", &id) == 1) {
                            if (cfg->num_ids < 16) cfg->ids[cfg->num_ids++] = id;
                        }
                        while (*id_str && *id_str != ',' && *id_str != ']') id_str++;
                        if (*id_str == ',') id_str++;
                    }
                } else {
                    int id;
                    if (sscanf(id_str, "%d", &id) == 1) {
                        if (id != -1) {
                            if (cfg->num_ids < 16) cfg->ids[cfg->num_ids++] = id;
                        }
                    }
                }
            } else if (strncmp(p, "port:", 5) == 0) {
                sscanf(p + 5, " %63s", cfg->port_or_can);
            } else if (strncmp(p, "can:", 4) == 0) {
                sscanf(p + 4, " %63s", cfg->port_or_can);
            } else if (strncmp(p, "baudrate:", 9) == 0) {
                sscanf(p + 9, " %d", &cfg->baudrate);
            } else if (strncmp(p, "nums:", 5) == 0) {
                sscanf(p + 5, " %d", &cfg->nums);
            }
        }
    }
    fclose(f);
    return 0;
}

static inline void load_config_and_args(int argc, char **argv,
                                        const char **driver,
                                        const char **iface_or_port,
                                        int *baudrate,
                                        int *ids, int *num_ids,
                                        int *nums) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--driver") == 0 && i + 1 < argc) {
            *driver = argv[i + 1];
        }
    }

    struct test_config cfg;
    if (parse_yaml_config("../config/config_parameters.yaml", *driver, &cfg) == 0 ||
        parse_yaml_config("../../config/config_parameters.yaml", *driver, &cfg) == 0 ||
        parse_yaml_config("components/peripherals/motor/config/config_parameters.yaml", *driver, &cfg) == 0) {
        if (iface_or_port && cfg.port_or_can[0] != '\0') {
            *iface_or_port = strdup(cfg.port_or_can);
        }
        if (baudrate && cfg.baudrate != -1) {
            *baudrate = cfg.baudrate;
        }
        if (ids && num_ids && cfg.num_ids > 0) {
            *num_ids = cfg.num_ids;
            for (int i = 0; i < cfg.num_ids; i++) {
                ids[i] = cfg.ids[i];
            }
        }
        if (nums && cfg.nums != -1) {
            *nums = cfg.nums;
        }
    }

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--driver") == 0 && i + 1 < argc) {
            i++;
        } else if ((strcmp(argv[i], "--if") == 0 || strcmp(argv[i], "--port") == 0) && i + 1 < argc) {
            if (iface_or_port) *iface_or_port = argv[++i];
        } else if (strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            if (baudrate) *baudrate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--nums") == 0 && i + 1 < argc) {
            if (nums) *nums = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--id") == 0 && i + 1 < argc) {
            if (ids && num_ids) {
                *num_ids = 0;
                char *p = argv[++i];
                char *token = strtok(p, ",");
                while (token && *num_ids < 16) {
                    ids[(*num_ids)++] = atoi(token);
                    token = strtok(NULL, ",");
                }
            }
        }
    }
}

#endif
