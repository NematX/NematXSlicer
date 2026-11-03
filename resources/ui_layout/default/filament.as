// Temperature controls operate on the filament preset being edited through the host bindings.

// Return the greater first-layer or other-layer temperature at vector index zero.
int s_easy_temp_get()
{
    int temp = get_int("temperature");
    int ftemp = get_int("first_layer_temperature");
    if (temp > ftemp) {
        return temp;
    }
    return ftemp;
}

// Broadcast a nozzle temperature within the native 0..1500 bounds, otherwise leave both options unchanged.
void s_easy_temp_set(int temp)
{
    if (temp < 0 || temp > 1500)
        return;
    set_int("temperature", temp);
    set_int("first_layer_temperature", temp);
}

// Return Custom, With glue or Without glue as index 0, 1 or 2; the output string is unused.
int s_bed_fil_fff_get(string &out get_val)
{
    int bed_temperature = get_int("bed_temperature");
    int fl_bed_temperature = get_int("first_layer_bed_temperature");
    if (bed_temperature == fl_bed_temperature) {
        if (bed_temperature == 130) {
            return 1;
        }
        if (bed_temperature == 170) {
            return 2;
        }
    }
    return 0;
}

// Select by index, ignoring the label; Custom restores selected-preset vectors and invalid indices do nothing.
void s_bed_fil_fff_set(string &in new_val, int idx)
{
    if (idx == 0) {
        back_initial_value("bed_temperature");
        back_initial_value("first_layer_bed_temperature");
    } else if (idx == 1) {
        set_int("bed_temperature", 130);
        set_int("first_layer_bed_temperature", 130);
    } else if (idx == 2) {
        set_int("bed_temperature", 170);
        set_int("first_layer_bed_temperature", 170);
    }
}
