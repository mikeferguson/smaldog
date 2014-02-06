/*
 * Copyright (c) 2014 Michael E. Ferguson.  All right reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 */

#ifndef SMALDOG_CLASSICAL_CRAWL_H_
#define SMALDOG_CLASSICAL_CRAWL_H_

#include <smaldog/robot_state.h>

namespace smaldog
{

/**
 *  \brief The classical crawl gait, as a simple planner.
 */
class ClassicalCrawl : public Planner
{

// classic left-front, right-hind, right-front, left-hind gait
// specify beta > 0.75?
// use the COG trajectory from Pongas 2007

};

}  // namespace smaldog

#endif  // SMALDOG_ROBOT_STATE_H_
