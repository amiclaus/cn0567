/*****************************************************************************
 * ADuCM3029_demo_cn0503.c
 *****************************************************************************/

#include <sys/platform.h>
#include <stdint.h>
#include "adi_initialize.h"

#include "cn0567.h"
#include "error.h"
#include "iio_adpd410x.h"
#include "iio_app.h"
#include "util.h"

int main(int argc, char *argv[])
{
	int32_t ret;
	struct cn0567_dev *cn0567;

	adi_initComponents();

	ret = cn0567_init(&cn0567);
	if(ret != SUCCESS)
		return FAILURE;

	uint32_t data[8];

	ret = adpd410x_set_opmode(cn0567->adpd4100_handler, ADPD410X_GOMODE);
	if (ret != SUCCESS)
		return ret;

	ret = adpd410x_get_data(cn0567->adpd4100_handler, data);
	if (ret != SUCCESS)
		return ret;

	ret = adpd410x_set_opmode(cn0567->adpd4100_handler, ADPD410X_STANDBY);
	if (ret != SUCCESS)
		return ret;


	struct iio_app_device devices[] = {
		IIO_APP_DEVICE("adpd410x", cn0567->adpd4100_handler,
			       &adpd410x_iio_descriptor,
			       NULL, NULL),
	};

	return iio_app_run(devices, ARRAY_SIZE(devices));

	return SUCCESS;
}
