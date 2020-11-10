/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#include "./CollisionDetectionPrimitive.h"
#include <Physics/RigidBody.h>
#include <Utils/Utils.h>


CollisionDetectionPrimitive::CollisionDetectionPrimitive(RigidBody* theBody){
	this->bdy = theBody;
	type = UNKNOWN_CDP;
}

CollisionDetectionPrimitive::~CollisionDetectionPrimitive(void){
}

/**
	draw an outline of the primitive...
*/
void CollisionDetectionPrimitive::draw(){

}

