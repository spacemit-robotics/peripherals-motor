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

#include <unistd.h>
#include <limits.h>

static inline int parse_yaml_config_search(const char *driver, struct test_config *cfg) {
    char path[PATH_MAX + 128];
    char temp_dir[PATH_MAX];

    // 1. Search upwards from Current Working Directory (CWD)
    if (getcwd(temp_dir, sizeof(temp_dir))) {
        while (1) {
            snprintf(path, sizeof(path), "%s/components/peripherals/motor/config/config_parameters.yaml", temp_dir);
            if (parse_yaml_config(path, driver, cfg) == 0) return 0;

            // Also check immediate config folder just in case
            snprintf(path, sizeof(path), "%s/config/config_parameters.yaml", temp_dir);
            if (parse_yaml_config(path, driver, cfg) == 0) return 0;

            char *last_slash = strrchr(temp_dir, '/');
            if (!last_slash || last_slash == temp_dir) break;
            *last_slash = '\0';
        }
    }

    // 2. Search upwards from Executable Directory (handles running via absolute path from outside SDK)
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len > 0) {
        exe_path[len] = '\0';
        char *last_slash = strrchr(exe_path, '/');
        if (last_slash) {
            *last_slash = '\0';
            strncpy(temp_dir, exe_path, sizeof(temp_dir));
            while (1) {
                snprintf(path, sizeof(path), "%s/components/peripherals/motor/config/config_parameters.yaml", temp_dir);
                if (parse_yaml_config(path, driver, cfg) == 0) return 0;

                last_slash = strrchr(temp_dir, '/');
                if (!last_slash || last_slash == temp_dir) break;
                *last_slash = '\0';
            }
        }
    }

    return -1;
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
    if (parse_yaml_config_search(*driver, &cfg) == 0) {
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
                char buf[256];
                strncpy(buf, argv[++i], sizeof(buf) - 1);
                buf[sizeof(buf) - 1] = '\0';
                char *token = strtok(buf, ",");
                while (token && *num_ids < 16) {
                    ids[(*num_ids)++] = atoi(token);
                    token = strtok(NULL, ",");
                }
            }
        }
    }
}

#endif
