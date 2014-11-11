/****************************************************************************
    Copyright (C) 2003 Alex Shepherd

    Portions Copyright (C) Digitrax Inc.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************

    IMPORTANT:

    Some of the message formats used in this code are Copyright Digitrax, Inc.
    and are used with permission as part of the EmbeddedLocoNet project. That
    permission does not extend to uses in other software products. If you wish
    to use this code, algorithm or these message formats outside of
    EmbeddedLocoNet, please contact Digitrax Inc, for specific permission.

    Note: The sale any LocoNet device hardware (including bare PCB's) that
    uses this or any other LocoNet software, requires testing and certification
    by Digitrax Inc. and will be subject to a licensing agreement.

    Please contact Digitrax Inc. for details.

*****************************************************************************

 Title :   LocoNet Utility Functions
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     24-Dec-2003
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides LocoNet utility functions
       
*****************************************************************************/

#include "utils.h"

void decodePeerData( peerXferMsg *pMsg, uint8_t *pOutData )
{
  uint8_t	Index ;
  uint8_t	Mask ;
  uint8_t	* pInData ;
  uint8_t	* pBits ;
  
  Mask = 0x01 ;
  pInData = &pMsg->d1 ;
  pBits = &pMsg->pxct1 ;

  for( Index = 0; Index < 8; Index++)
  {
	  pOutData[Index] = *pInData ;
    if( *pBits & Mask )
    	pOutData[Index] |= 0x80 ;
      
    if( Index == 3 )
    {
		  Mask = 0x01 ;
		  pInData = &pMsg->d5 ;
		  pBits = &pMsg->pxct2 ;
    }
    else
    {
    	Mask <<= (uint8_t) 1 ;
      pInData++;
    }
  }
}

void encodePeerData( peerXferMsg *pMsg, uint8_t *pInData )
{
  uint8_t	Index ;
  uint8_t	Mask ;
  uint8_t	* pOutData ;
  uint8_t	* pBits ;
  
  Mask = 0x01 ;
  pOutData = &pMsg->d1 ;
  pBits = &pMsg->pxct1 ;

  for( Index = 0; Index < 8; Index++)
  {
    *pOutData = pInData[Index] & (uint8_t)0x7F;	// fixed SBor040102
    if( pInData[Index] & 0x80 )
    	*pBits |= Mask ;
      
    if( Index == 3 )
    {
		  Mask = 0x01 ;
		  pOutData = &pMsg->d5 ;
		  pBits = &pMsg->pxct2 ;
    }
		else
    {
      Mask <<= (uint8_t) 1 ;
			pOutData++;    
    }
  }
}
