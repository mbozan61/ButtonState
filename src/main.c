/**
 * @file    main.c
 * @authors mario.bozan
 * @brief   Main file.
 * @details Button state implementation for PEP selection task. Nucleo F446RE board used.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define SW0_NODE             DT_ALIAS(sw0)  //!< User button B1 on Nucleo F446RE
#define DEBOUNCE_PRESS_MS    30             //!< Debounce delay for pressing button in ms
#define DEBOUNCE_RELEASE_MS  40             //!< Debounce delay for releasing button in ms
#define HIGH                 1              //!< High level
#define LOW                  0              //!< Low level

//! Nucleo F446RE user button
static const struct gpio_dt_spec usrButton = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});

//! Button state enum
typedef enum {
    BUTTON_PRESSED = 0,
    BUTTON_RELEASED,
    BUTTON_JUST_PRESSED,
    BUTTON_JUST_RELEASED,
    BUTTON_IDLE
} buttonStateType;

//! Button type structure
typedef struct {
    struct gpio_dt_spec gpioStruct;
    bool isPullUp;
    int lastButtonReading;
    buttonStateType lastButtonState;
    int32_t debounceTimer;
    bool debounceInProgress;
} buttonStructType;

// Private function decleration
static void initButton(buttonStructType *button, const struct gpio_dt_spec *gpioButton, bool buttonPolarity);
static bool ReadPin(const struct device *port, gpio_pin_t pin);
static buttonStateType GetButtonState(buttonStructType *btn, int32_t debouncePressTime, int32_t debounceReleaseTime);

/**
 * @brief   Initialize GPIO
 * @details This function initializes given GPIO (button).
 * @param   *button - pointer to buttonStructType structure.
 * @param   *gpioButton - pointer to gpio information structure.
 * @param   buttonPolarity - TRUE if GPIO is pulled up, FALSE if GPIO is pulled down.
 * @return  void
 */
void initButton(buttonStructType *button, const struct gpio_dt_spec *gpioButton, bool buttonPolarity) {
    int initPinState;
    // Initialize user button
	if (!gpio_is_ready_dt(gpioButton)) {
		printk("Error: button device %s is not ready\n",
            gpioButton->port->name);
		return;
	}
    // Set button structure fields
    button->gpioStruct = *gpioButton;
    // Set button polarity (logical level)
    button->isPullUp = buttonPolarity;
    // Read initial pin state
    initPinState = gpio_pin_get(gpioButton->port, gpioButton->pin);
    button->lastButtonReading = initPinState;
    // Set button state according to pin state
    if(initPinState == true) {
        button->lastButtonState = BUTTON_RELEASED;
    } else {
        button->lastButtonState = BUTTON_PRESSED;
    }
    // Initialize debounce values
    button->debounceTimer = 0;
    button->debounceInProgress = false;
}

/**
 * @brief   Read the state of the button.
 * @details This function reads gpio input level. It will return HIGH if the logical level
 *          is high and LOW if logical level is low. In this case, user button on Nucelo F446RE
 *          board is used. That button is pulled up so function will return HIGH when button is
 *          released and LOW when it is pressed.
 * @param   *port - pointer to device port
 * @param   pin   - gpio pin number
 * @return  HIGH (1) if logic level is high, LOW (0) if logical level is low
 */
static bool ReadPin(const struct device *port, gpio_pin_t pin) {
    if (gpio_pin_get(port, pin)) {
        return HIGH;
    } else if(!gpio_pin_get(port, pin)) {
        return LOW;
    } else {
        printk("Unable to read logical level! Returning 0.");
        return 0;
    }
}

/**
 * @brief   Button state machine.
 * @details This function implements state machine for different button states.
 * @param   *button - pointer to gpio information structure.
 * @param   debouncePressTime - debounce time for pressing the button
 * @param   debounceReleaseTime - debounce time for releasing the button
 * @return  buttonStateType enum type
 */
static buttonStateType GetButtonState(buttonStructType *btn, int32_t debouncePressTime, int32_t debounceReleaseTime) {
    bool buttonReading = ReadPin(btn->gpioStruct.port, btn->gpioStruct.pin);
    bool buttonPressState;
    bool lastButtonPressState;
    int32_t debounceTime;

    // Get button press state according to button polarity, needed to determine which debounce time to use
    if(btn->isPullUp == true) {
        buttonPressState = !buttonReading;
    } else {
        buttonPressState = buttonReading;
    }

    // Get last button press state according to button polarity (needed to determine if button state has changed)
    if(btn->isPullUp == true) {
        lastButtonPressState = !btn->lastButtonReading;
    } else {
        lastButtonPressState = btn->lastButtonReading;
    }

    // Get current time, neede to check if debounce time has passed
    int32_t currentTime = k_uptime_get();

    // Check if button state has changed
    if (buttonPressState != lastButtonPressState) {
        
        // Check if debounce is in progress
        if (!btn->debounceInProgress) {
            // Start debounce timer and return IDLE
            btn->debounceInProgress = true;
            btn->debounceTimer = currentTime;
            return BUTTON_IDLE;
        }

        // Determine which debounce time to use according to button press or release
        if(buttonPressState == true) {
            debounceTime = debouncePressTime;
        } else {
            debounceTime = debounceReleaseTime;
        }

        // Check if debounce time has passed
        if (currentTime - btn->debounceTimer >= debounceTime) {
            // If debounce time has passed, then update button structure field values
            btn->lastButtonReading = buttonReading;
            btn->debounceInProgress = false;

            // If the button was pressed, return JUST_PRESSED and vice versa
            // This is the first time the button was pressed, since the state of the button was just changed
            if (buttonPressState == true) {
                btn->lastButtonState = BUTTON_JUST_PRESSED;
            } else {
                btn->lastButtonState = BUTTON_JUST_RELEASED;
            }

            return btn->lastButtonState;
        }

        // If debounce time has not yet passed, return IDLE
        return BUTTON_IDLE;
    }

    // If there was no change in button state, but debounce is still in progress, return IDLE
    if (btn->debounceInProgress) {
        return BUTTON_IDLE;
    }
    
    // If there was no change in button state, then return PRESSED or RELEASED corresponding to "JUST" state
    if (btn->lastButtonState == BUTTON_JUST_PRESSED) {
        btn->lastButtonState = BUTTON_PRESSED;
    } else if (btn->lastButtonState == BUTTON_JUST_RELEASED) {
        btn->lastButtonState = BUTTON_RELEASED;
    }

    return btn->lastButtonState;
}


/**
 * @brief Main function
 */
int main(void) {
    buttonStructType button = {0};
    initButton(&button, &usrButton, true);
    buttonStateType buttonState;

    // Read button initial logical level
    bool buttonInitLogicalLevel = ReadPin(usrButton.port, usrButton.pin);
    bool buttonUpdatedLogicalLevel = buttonInitLogicalLevel;

    // Wait for the first press then go to state machine
    while(buttonUpdatedLogicalLevel == buttonInitLogicalLevel) {
        buttonUpdatedLogicalLevel = ReadPin(usrButton.port, usrButton.pin);
        printk("Button not yet pressed!\n");
    }
    
    while (true) {
        // Get button state from state machine
        buttonState = GetButtonState(&button, (int32_t)DEBOUNCE_PRESS_MS, (int32_t)DEBOUNCE_RELEASE_MS);
        // Print state
        switch(buttonState) {
            case BUTTON_PRESSED:
            printk("Button pressed!\n");
            break;
            case BUTTON_RELEASED:
            printk("Button released!\n");
            break;
            case BUTTON_JUST_PRESSED:
            printk("Button pressed for the first time!\n");
            break;
            case BUTTON_JUST_RELEASED:
            printk("Button released for the first time!\n");
            break;
            case BUTTON_IDLE:
            printk("Button idle, debouncing...\n");
            break;
            default:
            printk("Button state undefined!\n");
        }
    }

    return 0;
}
