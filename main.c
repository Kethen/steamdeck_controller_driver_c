/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <linux/input.h>
#include <linux/uinput.h>

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <endian.h>
#include <stdint.h>
#include <pthread.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <dirent.h>

#define BIT(data, bit) (data & (1 << bit))

// deck input state parsing
struct deck_input_state{
	uint32_t sequence_number; // 4-7

	// buttons
	uint8_t right_trigger_full_press; // 8.0
	uint8_t left_trigger_full_press; // 8.1
	uint8_t right_shoulder; // 8.2
	uint8_t left_shoulder; // 8.3
	uint8_t y; // 8.4
	uint8_t b; // 8.5
	uint8_t x; // 8.6
	uint8_t a; // 8.7
	uint8_t up; // 9.0
	uint8_t right; // 9.1
	uint8_t left; // 9.2
	uint8_t down; // 9.3
	uint8_t left_menu; // 9.4
	uint8_t steam_logo; // 9.5
	uint8_t right_menu; // 9.6
	uint8_t left_bottom_grip; // 9.7
	uint8_t right_bottom_grip; // 10.0
	uint8_t left_pad_press; // 10.1
	uint8_t right_pad_press; // 10.2
	uint8_t left_pad_touch; // 10.3
	uint8_t right_pad_touch; // 10.4
	uint8_t left_stick_click; // 10.6
	uint8_t right_stick_click; // 11.2
	uint8_t left_top_grip; // 13.1
	uint8_t right_top_grip; // 13.2
	uint8_t left_stick_touch; // 13.6
	uint8_t right_stick_touch; // 13.7
	uint8_t quick_access; // 14.2

	// vectors
	int16_t left_pad_x; // 16-17 le
	int16_t left_pad_y; // 18-19 le
	int16_t right_pad_x; // 20-21 le
	int16_t right_pad_y; // 22-23 le
	int16_t accelerometer_x; // 24-25 le
	int16_t accelerometer_y; // 26-27 le
	int16_t accelerometer_z; // 28-29 le
	int16_t gyro_x; // 30-31 le
	int16_t gyro_y; // 32-33 le
	int16_t gyro_z; // 34-35 le
	int16_t quaternion_w; // 36-37 le
	int16_t quaternion_x; // 38-39 le
	int16_t quaternion_y; // 40-41 le
	int16_t quaternion_z; // 42-43 le
	uint16_t left_trigger; // 44-45 le
	uint16_t right_trigger; // 46-47 le
	int16_t left_stick_x; // 48-49 le
	int16_t left_stick_y; // 50-51 le
	int16_t right_stick_x; // 52-53 le
	int16_t right_stick_y; // 54-55 le
	uint16_t left_pad_pressure; // 56-57
	uint16_t right_pad_pressure; // 58-59
};

int16_t le16_to_host_cropped(uint8_t *data){
	uint16_t buf;
	int16_t *p = &buf;
	buf = le16toh(*(uint16_t *)data);
	if(*p == -32768){
		*p = -32767;
	}
	return *p;
}

uint16_t unsigned_le16_to_host(uint8_t *data){
	return le16toh(*(uint16_t *)data);
}

uint32_t unsigned_le32_to_host(uint8_t *data){
	return le16toh(*(uint32_t *)data);
}

void parse_deck_input_state(uint8_t *data, struct deck_input_state* output){
	output->sequence_number = unsigned_le32_to_host(&(data[4]));

	output->right_trigger_full_press = BIT(data[8], 0);
	output->left_trigger_full_press = BIT(data[8], 1);
	output->right_shoulder = BIT(data[8], 2);
	output->left_shoulder = BIT(data[8], 3);
	output->y = BIT(data[8], 4);
	output->b = BIT(data[8], 5);
	output->x = BIT(data[8], 6);
	output->a = BIT(data[8], 7);
	output->up = BIT(data[9], 0);
	output->right = BIT(data[9], 1);
	output->left = BIT(data[9], 2);
	output->down = BIT(data[9], 3);
	output->left_menu = BIT(data[9], 4);
	output->steam_logo = BIT(data[9], 5);
	output->right_menu = BIT(data[9], 6);
	output->left_bottom_grip = BIT(data[9], 7);
	output->right_bottom_grip = BIT(data[10], 0);
	output->left_pad_press = BIT(data[10], 1);
	output->right_pad_press = BIT(data[10], 2);
	output->left_pad_touch = BIT(data[10], 3);
	output->right_pad_touch = BIT(data[10], 4);
	output->left_stick_click = BIT(data[10], 6);
	output->right_stick_click = BIT(data[11], 2);
	output->left_top_grip = BIT(data[13], 1);
	output->right_top_grip = BIT(data[13], 2);
	output->left_stick_touch = BIT(data[13], 6);
	output->right_stick_touch = BIT(data[13], 7);
	output->quick_access = BIT(data[14], 2);

	output->left_pad_x = le16_to_host_cropped(&(data[16]));
	output->left_pad_y = le16_to_host_cropped(&(data[18]));
	output->right_pad_x = le16_to_host_cropped(&(data[20]));
	output->right_pad_y = le16_to_host_cropped(&(data[22]));
	output->accelerometer_x = le16_to_host_cropped(&(data[24]));
	output->accelerometer_y = le16_to_host_cropped(&(data[26]));
	output->accelerometer_z = le16_to_host_cropped(&(data[28]));
	output->gyro_x = le16_to_host_cropped(&(data[30]));
	output->gyro_y = le16_to_host_cropped(&(data[32]));
	output->gyro_z = le16_to_host_cropped(&(data[34]));
	output->quaternion_w = le16_to_host_cropped(&(data[36]));
	output->quaternion_x = le16_to_host_cropped(&(data[38]));
	output->quaternion_y = le16_to_host_cropped(&(data[40]));
	output->quaternion_z = le16_to_host_cropped(&(data[42]));
	output->left_trigger = unsigned_le16_to_host(&(data[44]));
	output->right_trigger = unsigned_le16_to_host(&(data[46]));
	output->left_stick_x = le16_to_host_cropped(&(data[48]));
	output->left_stick_y = le16_to_host_cropped(&(data[50]));
	output->right_stick_x = le16_to_host_cropped(&(data[52]));
	output->right_stick_y = le16_to_host_cropped(&(data[54]));
	output->left_pad_pressure = unsigned_le16_to_host(&(data[56]));
	output->right_pad_pressure = unsigned_le16_to_host(&(data[58]));
}

#define PRINT_STATE_BOOL(state, item) printf(#item ": %s\n", state->item ? "on": "off");
#define PRINT_STATE_INT16(state, item) {int val = state->item; printf(#item ": %d\n", val);}
#define PRINT_STATE_UINT16(state, item) {unsigned int val = state->item; printf(#item ": %d\n", val);}

void print_deck_input_state(struct deck_input_state *state){
	printf("----\n");
	PRINT_STATE_BOOL(state, right_trigger_full_press)
	PRINT_STATE_BOOL(state, left_trigger_full_press)
	PRINT_STATE_BOOL(state, right_shoulder)
	PRINT_STATE_BOOL(state, left_shoulder)
	PRINT_STATE_BOOL(state, y)
	PRINT_STATE_BOOL(state, b)
	PRINT_STATE_BOOL(state, x)
	PRINT_STATE_BOOL(state, a)
	PRINT_STATE_BOOL(state, up)
	PRINT_STATE_BOOL(state, right)
	PRINT_STATE_BOOL(state, left)
	PRINT_STATE_BOOL(state, down)
	PRINT_STATE_BOOL(state, left_menu)
	PRINT_STATE_BOOL(state, steam_logo)
	PRINT_STATE_BOOL(state, right_menu)
	PRINT_STATE_BOOL(state, left_bottom_grip)
	PRINT_STATE_BOOL(state, right_bottom_grip)
	PRINT_STATE_BOOL(state, left_pad_press)
	PRINT_STATE_BOOL(state, right_pad_press)
	PRINT_STATE_BOOL(state, left_pad_touch)
	PRINT_STATE_BOOL(state, right_pad_touch)
	PRINT_STATE_BOOL(state, left_stick_click)
	PRINT_STATE_BOOL(state, right_stick_click)
	PRINT_STATE_BOOL(state, left_top_grip)
	PRINT_STATE_BOOL(state, right_top_grip)
	PRINT_STATE_BOOL(state, left_stick_touch)
	PRINT_STATE_BOOL(state, right_stick_touch)
	PRINT_STATE_BOOL(state, quick_access)

	PRINT_STATE_INT16(state, left_pad_x)
	PRINT_STATE_INT16(state, left_pad_y)
	PRINT_STATE_INT16(state, right_pad_x)
	PRINT_STATE_INT16(state, right_pad_y)
	PRINT_STATE_INT16(state, accelerometer_x)
	PRINT_STATE_INT16(state, accelerometer_y)
	PRINT_STATE_INT16(state, accelerometer_z)
	PRINT_STATE_INT16(state, gyro_x)
	PRINT_STATE_INT16(state, gyro_y)
	PRINT_STATE_INT16(state, gyro_z)
	PRINT_STATE_INT16(state, quaternion_w)
	PRINT_STATE_INT16(state, quaternion_x)
	PRINT_STATE_INT16(state, quaternion_y)
	PRINT_STATE_INT16(state, quaternion_z)
	PRINT_STATE_UINT16(state, left_trigger)
	PRINT_STATE_UINT16(state, right_trigger)
	PRINT_STATE_INT16(state, left_stick_x)
	PRINT_STATE_INT16(state, left_stick_y)
	PRINT_STATE_INT16(state, right_stick_x)
	PRINT_STATE_INT16(state, right_stick_y)
	PRINT_STATE_UINT16(state, left_pad_pressure)
	PRINT_STATE_UINT16(state, right_pad_pressure)
	printf("----\n");
}

// deck controller mode settings
int set_register(int fd, uint8_t reg, uint16_t value){
	uint8_t buf[5] = {0};
	buf[0] = 0x87;
	buf[1] = 0x3;
	buf[2] = reg;
	*(uint16_t *)(&(buf[3])) = htobe16(value);
	return ioctl(fd, HIDIOCSFEATURE(5), buf);
}

int send_single_byte(int fd, uint8_t byte){
	uint8_t buf[3] = {0};
	buf[1] = byte;
	return ioctl(fd, HIDIOCSFEATURE(3), buf);
}

int toggle_lizard_mode_mapping(int fd, uint8_t enabled){
	if(enabled){
		// reset lizard mode input device mapping
		int res = send_single_byte(fd, 0x85);
		if(res < 0){
			return res;
		}
		// restore all registers
		res = send_single_byte(fd, 0x8e);
		if(res < 0){
			return res;
		}
		// enable smooth mouse?
		res = set_register(fd, 0x18, 0x01);
		if(res < 0){
			return res;
		}
	}else{
		// clear lizard mode input devices mapping
		int res = send_single_byte(fd, 0x81);
		if(res < 0){
			return res;
		}
		// disable mouse smoothing
		res = set_register(fd, 0x18, 0);
		if(res < 0){
			return res;
		}
		// disable trackpad clicks
		res = set_register(fd, 0x34, 0xffff);
		if(res < 0){
			return res;
		}
		res = set_register(fd, 0x35, 0xffff);
		if(res < 0){
			return res;
		}
		// disable trackpad
		res = set_register(fd, 0x07, 0x07);
		if(res < 0){
			return res;
		}
		res = set_register(fd, 0x08, 0x07);
		if(res < 0){
			return res;
		}
	}
	return 0;
}

// probing
int find_deck_hidraw(char *path_buf){
	DIR *dev_dir = opendir("/dev");
	if(dev_dir == 0){
		return -1;
	}
	struct dirent *dir;
	int ret = -1;
	while(dir = readdir(dev_dir)){
		const char *const match = "hidraw";
		if(strncmp(match, dir->d_name, 6)){
			continue;
		}else{
			char full_path[5 + strlen(dir->d_name) + 1];
			sprintf(full_path, "/dev/%s", dir->d_name);
			int fd = open(full_path, O_RDWR);
			if(fd < 0){
				fprintf(stderr, "warning: cannot open %s for ioctl\n", full_path);
				continue;
			}

			struct hidraw_devinfo info;
			memset(&info, 0x0, sizeof(info));
			int res = ioctl(fd, HIDIOCGRAWINFO, &info);
			if(res < 0){
				fprintf(stderr, "warning: failed grabbing raw info from %s\n", full_path);
				close(fd);
				continue;
			}

			if(info.vendor != 0x28de || info.product != 0x1205){
				fprintf(stderr, "info: %s is not a steam deck controller device\n", full_path);
				close(fd);
				continue;
			}

			int desc_size;
			struct hidraw_report_descriptor rpt_desc;
			res = ioctl(fd, HIDIOCGRDESCSIZE, &desc_size);
			if(res < 0){
				fprintf(stderr, "warning: failed grabbing report descriptor size from %s\n", full_path);
				close(fd);
				continue;
			}else{
				memset(&rpt_desc, 0x0, sizeof(rpt_desc));
				rpt_desc.size = desc_size;
				if(desc_size < 4){
					close(fd);
					continue;
				}
				res = ioctl(fd, HIDIOCGRDESC, &rpt_desc);
				if(res < 0){
					fprintf(stderr, "warning: failed grabbing report descriptor from %s\n", full_path);
					close(fd);
					continue;
				}else{
					const char mouse[] = {0x5, 0x1, 0x9, 0x2};
					const char keyboard[] = {0x5, 0x1, 0x9, 0x6};
					if(!memcmp(mouse, rpt_desc.value, 4)){
						fprintf(stderr, "info: %s is lizard mode mouse", full_path);
						close(fd);
						continue;
					}
					if(!memcmp(keyboard, rpt_desc.value, 4)){
						fprintf(stderr, "info: %s is lizard mode keyboard", full_path);
						close(fd);
						continue;
					}
					// it'll be deck controller after that, unless there's a firmware change
					strcpy(path_buf, full_path);
					ret = 0;
					close(fd);
					break;
				}
			}
		}
	}
	closedir(dev_dir);
	return ret;
}

int find_and_open_deck_hidraw(){
	char path_buf[256];
	if(!find_deck_hidraw(path_buf)){
		int fd = open(path_buf, O_RDWR);
		if(fd >= 0){
			return fd;
		}
	}
	return -1;
}

// thread loops
struct lizard_mode_thread_args{
	uint8_t stop;
};
void *lizard_mode_thread(struct lizard_mode_thread_args *args){
	int fd = -1;
	while(!args->stop){
		if(fd < 0){
			usleep(1000 * 1000);
			fprintf(stderr, "info: probing for steam deck controller hidraw on lizard mode disabling thread\n");
			fd = find_and_open_deck_hidraw();
			continue;
		}
		int res = toggle_lizard_mode_mapping(fd, 0);
		if(res < 0){
			fprintf(stderr, "warning: failed setting lizard mode\n");
			close(fd);
			fd = -1;
		}
		usleep(200 * 1000); // 5 hz
	}
	if(fd >= 0){
		close(fd);
	}
	pthread_exit(0);
}

struct deck_input_state_poller_thread_args{
	uint8_t stop;
	uint8_t drop;
	pthread_mutex_t lock;
	struct deck_input_state state;
	uint8_t seq;
};
void *deck_input_state_poller_thread(struct deck_input_state_poller_thread_args *args){
	int fd = -1;
	while(!args->stop){
		if(fd < 0){
			usleep(1000 * 1000);
			fprintf(stderr, "info: probing for steam deck controller hidraw on poller thread\n");
			fd = find_and_open_deck_hidraw();
			continue;
		}
		uint8_t read_buf[256];
		int read_size = read(fd, read_buf, 256);
		if(read_size == -1){
			fprintf(stderr, "warning: error when reading from opened hidraw device\n");
			close(fd);
			continue;
		}
		if(args->drop){
			continue;
		}
		if(read_size != 64){
			fprintf(stderr, "warning: packet is somehow not 64 byte, read %d\n", read_size);
			continue;
		}
		if(read_buf[0] != 0x1 || read_buf[1] != 0x0){
			fprintf(stderr, "warning: packet is somehow not led by [0x1, 0x0]\n");
			continue;
		}
		if(read_buf[2] != 0x9){
			fprintf(stderr, "warning: packet is not of deck input data\n");
			continue;
		}
		if(pthread_mutex_trylock(&args->lock) != 0){
			// something is reading the state, just drop the input event
			continue;
		}
		parse_deck_input_state(read_buf, &args->state);
		args->seq++;
		pthread_mutex_unlock(&args->lock);
	}
	if(fd >= 0){
		close(fd);
	}
	pthread_exit(0);
}

struct controller_uinput_thread_args{
	uint8_t stop;
	pthread_mutex_t *lock;
	struct deck_input_state *state;
	uint8_t *seq;
};
int emit(int fd, int type, int code, int val){
	struct input_event ie;
	ie.type = type;
	ie.code = code;
	ie.value = val;
	ie.time.tv_sec = 0;
	ie.time.tv_usec = 0;
	return write(fd, &ie, sizeof(ie));
}
int emit_btn(int fd, int code, int val){
	int res = emit(fd, EV_KEY, code, val);
	if(res < 0){
		return res;
	}
	int res2 = emit(fd, EV_SYN, SYN_REPORT, 0);
	if(res2 < 0){
		return res2;
	}
	return res + res2;
}
int emit_abs(int fd, int code, int val){
	int res = emit(fd, EV_ABS, code, val);
	if(res < 0){
		return res;
	}
	int res2 = emit(fd, EV_SYN, SYN_REPORT, 0);
	if(res2 < 0){
		return res2;
	}
	return res + res2;
}
void *controller_uinput_thread(struct controller_uinput_thread_args *args){
	struct deck_input_state last_state;
	struct deck_input_state state;
	memset(&last_state, 0, sizeof(last_state));
	int fd = -1;
	uint8_t last_seq;
	uint8_t same_seq_cnt = 0;
	while(!args->stop){
		if(fd < 0){
			usleep(1000 * 1000);
			int staging_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
			if(staging_fd < 0){
				continue;
			}
			if(
				ioctl(staging_fd, UI_SET_EVBIT, EV_KEY) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_SOUTH) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_EAST) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_NORTH) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_WEST) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_TL) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_TR) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_SELECT) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_START) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_MODE) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_THUMBL) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_THUMBR) /* ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_DPAD_UP) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_DPAD_DOWN) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_DPAD_LEFT) ||
				ioctl(staging_fd, UI_SET_KEYBIT, BTN_DPAD_RIGHT ) */
			)
			{
				fprintf(stderr, "warning: failed setting up uinput btn\n");
				close(staging_fd);
				continue;
			}
			if(
				ioctl(staging_fd, UI_SET_EVBIT, EV_ABS) ||
				ioctl(staging_fd, UI_SET_ABSBIT, ABS_X) ||
				ioctl(staging_fd, UI_SET_ABSBIT, ABS_Y) ||
				ioctl(staging_fd, UI_SET_ABSBIT, ABS_Z) ||
				ioctl(staging_fd, UI_SET_ABSBIT, ABS_RX) ||
				ioctl(staging_fd, UI_SET_ABSBIT, ABS_RY) ||
				ioctl(staging_fd, UI_SET_ABSBIT, ABS_RZ) ||
				ioctl(staging_fd, UI_SET_ABSBIT, ABS_HAT0X) ||
				ioctl(staging_fd, UI_SET_ABSBIT, ABS_HAT0Y)
			)
			{
				fprintf(stderr, "warning: failed setting up uinput abs\n");
				close(staging_fd);
				continue;
			}
			#define SETUP_ABS(c, l, h, v) { \
				struct uinput_abs_setup abs_setup; \
				memset(&abs_setup, 0, sizeof(abs_setup)); \
				abs_setup.code = c; \
				abs_setup.absinfo.minimum = l; \
				abs_setup.absinfo.maximum = h; \
				abs_setup.absinfo.value = v; \
				if(ioctl(staging_fd, UI_ABS_SETUP, &abs_setup)){ \
					fprintf(stderr, "warning: failed setting up " #c "\n"); \
					close(staging_fd); \
					continue; \
				} \
			}
			SETUP_ABS(ABS_X, -32767, 32767, 0)
			SETUP_ABS(ABS_Y, -32767, 32767, 0)
			SETUP_ABS(ABS_Z, 0, 32767, 0)
			SETUP_ABS(ABS_RX, -32767, 32767, 0)
			SETUP_ABS(ABS_RY, -32767, 32767, 0)
			SETUP_ABS(ABS_RZ, 0, 32767, 0)
			SETUP_ABS(ABS_HAT0X, -1, 1, 0)
			SETUP_ABS(ABS_HAT0Y, -1, 1, 0)
			struct uinput_setup usetup;
			memset(&usetup, 0, sizeof(usetup));
			usetup.id.bustype = BUS_USB;
			usetup.id.vendor = 0x046d;
			usetup.id.product = 0xc21d;
			strcpy(usetup.name, "virtual x360 controller from steam deck controller");
			if(ioctl(staging_fd, UI_DEV_SETUP, &usetup) || ioctl(staging_fd, UI_DEV_CREATE)){
				fprintf(stderr, "warning: failed finishing uinput device creation\n");
				close(staging_fd);
				continue;
			}
			fd = staging_fd;
		}

		pthread_mutex_lock(args->lock);
		if(last_seq == *args->seq){
			pthread_mutex_unlock(args->lock);
			same_seq_cnt++;
			if(same_seq_cnt >= 125){
				same_seq_cnt = 125;
				usleep(25 * 1000); // drop to 40 Hz after idling for 1 second
			}
			continue;
		}
		same_seq_cnt = 0;
		last_seq = *args->seq;
		state = *args->state;
		pthread_mutex_unlock(args->lock);
		#define CHECK_BTN(v, b) { \
			if(last_state.v != state.v){ \
				if(emit_btn(fd, b, state.v ? 1 : 0) < 0){ \
					close(fd); \
					fd = -1; \
					continue; \
				} \
			} \
		}
		CHECK_BTN(a, BTN_SOUTH)
		CHECK_BTN(b, BTN_EAST)
		CHECK_BTN(x, BTN_NORTH)
		CHECK_BTN(y, BTN_WEST)
		CHECK_BTN(left_shoulder, BTN_TL)
		CHECK_BTN(right_shoulder, BTN_TR)
		CHECK_BTN(left_menu, BTN_SELECT)
		CHECK_BTN(right_menu, BTN_START)
		CHECK_BTN(steam_logo, BTN_MODE)
		CHECK_BTN(left_stick_click, BTN_THUMBL)
		CHECK_BTN(right_stick_click, BTN_THUMBR)
		/*
		CHECK_BTN(up, BTN_DPAD_UP)
		CHECK_BTN(down, BTN_DPAD_DOWN)
		CHECK_BTN(left, BTN_DPAD_LEFT)
		CHECK_BTN(right, BTN_DPAD_RIGHT)
		*/

		#define CHECK_ABS(v, a, m, o) { \
			if(last_state.v != state.v){ \
				if(emit_abs(fd, a, state.v * m + (o)) < 0){ \
					close(fd); \
					fd = -1; \
					continue; \
				} \
			} \
		}
		CHECK_ABS(left_stick_x, ABS_X, 1, 0)
		CHECK_ABS(left_stick_y, ABS_Y, -1, 0)
		CHECK_ABS(left_trigger, ABS_Z, 1, 0)
		CHECK_ABS(right_stick_x, ABS_RX, 1, 0)
		CHECK_ABS(right_stick_y, ABS_RY, -1, 0)
		CHECK_ABS(right_trigger, ABS_RZ, 1, 0)

		#define CHECK_HAT(vn, vp, a) { \
			if(last_state.vn != state.vn || last_state.vp != state.vp){ \
				int value = 0; \
				if(state.vn){ \
					value -= 1; \
				} \
				if(state.vp){ \
					value += 1; \
				} \
				if(emit_abs(fd, a, value) < 0){ \
					close(fd); \
					fd = -1; \
					continue; \
				} \
			} \
		}
		CHECK_HAT(up, down, ABS_HAT0Y)
		CHECK_HAT(left, right, ABS_HAT0X)

		last_state = state;
		usleep(8 * 1000); // 125 hz
	}
	pthread_exit(0);
}

int main(int argc, char **argv){
	char path_buf[256];
	int res = find_deck_hidraw(path_buf);
	if(res < 0){
		fprintf(stderr, "cannot find steam deck controller hidraw\n");
		exit(1);
	}

	printf("steam deck controller hidraw found at %s\n", path_buf);

	struct deck_input_state_poller_thread_args poller_args;
	poller_args.stop = 0;
	poller_args.drop = 0;
	pthread_mutex_init(&poller_args.lock, 0);
	poller_args.seq = 0;
	pthread_t poller_tid;
	res = pthread_create(&poller_tid, 0, (void * (*)(void *))deck_input_state_poller_thread, &poller_args);
	if(res != 0){
		fprintf(stderr, "cannot create poller thread\n");
		exit(1);
	}

	int uinput_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if(uinput_fd < 0){
		fprintf(stderr, "cannot open /dev/uinput\n");
		exit(1);
	}
	close(uinput_fd);
	struct controller_uinput_thread_args controller_args;
	controller_args.stop = 0;
	controller_args.lock = &poller_args.lock;
	controller_args.state = &poller_args.state;
	controller_args.seq = &poller_args.seq;
	pthread_t controller_thread;
	res = pthread_create(&controller_thread, 0, (void * (*)(void *))controller_uinput_thread, &controller_args);
	if(res != 0){
		fprintf(stderr, "cannot create controller uinput thread\n");
		exit(1);
	}

	struct lizard_mode_thread_args lizard_args;
	lizard_args.stop = 0;
	pthread_t lizard_tid;
	res = pthread_create(&lizard_tid, 0, (void * (*)(void *))lizard_mode_thread, &lizard_args);
	if(res != 0){
		fprintf(stderr, "cannot create lizard mode disabling thread\n");
		exit(1);
	}

	uint8_t read_buf[256];
	int read_size;
	struct deck_input_state state;
	int last_seq = 0;
	int same_seq_cnt = 0;
	pthread_join(poller_tid, 0);
	pthread_join(lizard_tid, 0);
	while(1){
		pthread_mutex_lock(&poller_args.lock);
		if(last_seq == poller_args.seq){
			pthread_mutex_unlock(&poller_args.lock);
			same_seq_cnt++;
			if(same_seq_cnt >= 100){
				usleep(100 * 1000);
			}
			continue;
		}
		same_seq_cnt = 0;
		print_deck_input_state(&poller_args.state);
		pthread_mutex_unlock(&poller_args.lock);
		usleep(8 * 1000);
	}
}
