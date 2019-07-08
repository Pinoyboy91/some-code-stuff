#include "RasterSurface.h"
#include "LabTiles.h"
#include "teleport.h"
#include <time.h>
#include <random>
#include "XTime.h"

#define SB 0xFF000000
#define SG 0x00FF0000
#define SR 0x0000FF00
#define SA 0x000000FF

#define DA 0xFF000000
#define DR 0x00FF0000
#define DG 0x0000FF00
#define DB 0x000000FF

static unsigned int *RSurf;
static unsigned int *TempSurf;
int curx;
int cury;

//10 is placement number, something greater than 0;
int counter = 10;

//source
unsigned int sb;
unsigned int sg;
unsigned int sr;
unsigned int sa;

//dest
unsigned int da;
unsigned int dr;
unsigned int dg;
unsigned int db;

float ratio;

unsigned int size;
bool temp = false;

unsigned int ColorLerp(unsigned int s, unsigned int d)
{
	// splitting up the channels
	sb = s & SB;
	sg = s & SG;
	sr = s & SR;
	sa = s & SA;
	

	da = d & DA;
	dr = d & DR;
	dg = d & DG;
	db = d & DB;
	ratio = (float)sa / 255.0f;


	//source
	sb = sb >> 24;
	sg = sg >> 16;
	sr = sr >> 8;
	//sa does not need to move

	da = da >> 24;
	dr = dr >> 16;
	dg = dg >> 8;
	//db does not need to move

	//blending
	sa = ((int)sa - (int)da) * ratio + (int)da;
	sr = ((int)sr - (int)dr) * ratio + (int)dr;
	sg = ((int)sg - (int)dg) * ratio + (int)dg;
	sb = ((int)sb - (int)db) * ratio + (int)db;

	

	//conversion
	sg = sg << 8;
	sr = sr << 16;
	sa = sa << 24;
	
	unsigned int newcolor = sa | sr | sg | sb;
	return newcolor;
}

void Reset()
{
	for (int i = 0; i < size; i++)
	{
		RSurf[i] = 0x00000000;
	}
}


void Blit(unsigned int sx1, unsigned int sy1, unsigned int sx2, unsigned int sy2, int swidth, int dwidth, unsigned int dx, unsigned int dy, const unsigned int sprites[])
{
	int temp = dx;
	for (int y = sy1; y < sy2; y++)
	{
		dx = temp;
		for (int x = sx1; x < sx2; x++)
		{
			if (dx > 500)
			{
				break;
			}
			RSurf[dwidth*dy + dx] = ColorLerp(sprites[swidth *y + x], RSurf[dwidth*dy + dx]);
			dx++;
		}
		dy++;
		if (dy > 500)
		{
			break;
		}
	}
}
void DrawBackground(int sx1, int sy1, int sx2, int sy2, int swidth, int dwidth)
{
	int width = abs(sx2 - sx1);
	int height = abs(sy2 - sy1);
	for (int i = 0; i < dwidth; i++)
	{
		for (int j = 0; j < dwidth; j++)
		{
			Blit(sx1, sy1, sx2, sy2, 400, 500, j, i, LabTiles_pixels);
			j += width-1;
		}
		i += height-1;
	}
}

void Animate()
{
	Blit(curx * 128, cury * 128, ((curx + 1) * 128), ((cury + 1) * 128), 1024, 500, 200, 200, teleport_pixels);

	if (counter > 1)
	{
		Blit(curx * 128, cury * 128, ((curx + 1) * 128), ((cury + 1) * 128), 1024, 500, 200, 200, teleport_pixels);
		counter = 0;
		curx++;
		if (curx > 7)
		{
			curx = 0;
			cury++;
			if (cury > 7)
			{
				cury = 0;
			}
		}
	}
	counter++;
}

int main(void)
{
	//XTime timer;
	RS_Initialize(500, 500);
	size = 500 * 500;
	RSurf = new unsigned int[size]();
	TempSurf = new unsigned int[16384]();
	
	

	//randomize coordinates
	srand(time(NULL));
	int x[10] = { 0 };
	int y[10] = { 0 };
	for (int i = 0; i < 10; i++)
	{

		x[i] = rand() % 450;
	}
	for (int i = 0; i < 10; i++)
	{

		y[i] = rand() % 450;
	}


	DrawBackground(280, 120, 326, 166, 400, 500);

	while (RS_Update(RSurf, 250000))
	{

		if (temp == true)
		{
			for (unsigned int y = 0; y < 128; y++)
			{
				for (unsigned int x = 0; x < 128; x++)
				{
					RSurf[500 * (200 + y) + (200 + x)] = TempSurf[128 * y + x];
				}
			}
		}
		for (int i = 0; i < 10; i++)
		{
			Blit(321, 17, 382, 94, 400, 500, x[i], y[i], LabTiles_pixels);
		}
		for (unsigned int y = 0; y < 128; y++)
		{
			for (unsigned int x = 0; x < 128; x++)
			{
				TempSurf[128 * y + x] = RSurf[500 * (200 + y) + (200 + x)];
				temp = true;
			}
		}
		Animate();
		
	}
	
	RS_Shutdown();
}


