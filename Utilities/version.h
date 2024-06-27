/*******************************************************************************
* Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#ifndef UTILITIES_VERSION_H_
#define UTILITIES_VERSION_H_


#define xstr(s) str(s)
#define str(s) 		#s
//
#define VER_MAJ		0
#define VER_MIN		8
#define VER_PATCH	1
#define VER_BUILD	0
// Date
#define VER_YEAR	2022
#define VER_MTH		12
#define VER_DAY		21

//
#define FIRMWARE_NAME		"MRD106"
#define FIRMWARE_VERSION	"v" xstr(VER_MAJ) "." xstr(VER_MIN) "." xstr(VER_PATCH)
#define FIRMWARE_INFO		(FIRMWARE_NAME "_" FIRMWARE_VERSION)

#define PLATFORM_NAME		"SmartSensor_MAX32666"
#define PLATFORM_TYPE		"HSP4.0"

#endif /* UTILITIES_VERSION_H_ */
