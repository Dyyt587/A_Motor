#include "workchain.h"

void wkc_init(wkc_t*wkc)
{
}
bool check_licence(wkc_t *wc, LicenseDomain licence)
{
	if((wc->lic_aprove.all & licence.all) == licence.all)
		return true;
	return false;
}
#define WKC_HANDLE_CHECK(x) if (x->handle)
// #define WKC_HANDLE_CHECK(x)
int wkc_handle(wkc_t *wkc)
{
	wkc_work_t *work = wkc->works;

	while (work)
	{
		if (check_licence(wkc, work->licence))
		{
			if (++work->trig_cnt > work->trig_level)
			{
				WKC_HANDLE_CHECK(work)
				{
					work->handle(wkc);
				}
				work->trig_cnt = 0;
			}
		}

		work = work->next;
	}
	return 0;
}
void wkc_work_add(wkc_t *wkc, wkc_work_t *work)
{
	wkc_work_t *works = wkc->works;
	if (wkc && work)
	{
		work->next = 0;
		if (works)
		{
			while (works->next)
				works = works->next;
		}
		else
		{
			wkc->works = work;
			return;
		}
		works->next = work;
	}
}