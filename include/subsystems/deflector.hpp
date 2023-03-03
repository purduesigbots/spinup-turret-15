#pragma once

namespace deflector {

/**
 * Puts the deflector in the up position
 */
void up();

/**
 * Puts the deflector in the down position
 */
void down();

/**
 * Toggle the deflector's position 
 */
void toggle();

/**
 * Returns whether the deflector is up or not
 * 
 * @return True if the deflector is the up position 
 */
bool is_up();

} // namespace deflector