
/* "Copyright (c) 2000-2003 The Regents of the University of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement
 * is hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
 * OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 */

/**
 * @author Joe Polastre
 */

interface HplMsp430Interrupt
{
  /** 
   * Enables MSP430 hardware interrupt on a particular port.
   */
  async command void enable();

  /** 
   * Disables MSP430 hardware interrupt on a particular port.
   */
  async command void disable();

  /** 
   * Clears the MSP430 Interrupt Pending Flag for a particular port.
   */
  async command void clear();

  /** 
   * Gets the current value of the input voltage of a port.
   *
   * @return TRUE if the pin is set high, FALSE if it is set low.
   */
  async command bool getValue();

  /** 
   * Sets whether the edge should be high to low or low to high.
   *
   * @param TRUE if the interrupt should be triggered on a low to high
   *        edge transition, false for interrupts on a high to low transition.
   */
  async command void edge(bool low_to_high);

  /**
   * Signalled when an interrupt occurs on a port.
   */
  async event void fired();
}

