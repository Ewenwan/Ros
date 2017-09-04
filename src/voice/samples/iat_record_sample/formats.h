#ifndef FORMATS_H_160601_TT
#define FORMATS_H_160601_TT		1

#ifndef WAVE_FORMAT_PCM  
#define WAVE_FORMAT_PCM  1
typedef struct tWAVEFORMATEX {
	unsigned short	  wFormatTag;
	unsigned short    nChannels;
	unsigned int      nSamplesPerSec;
	unsigned int      nAvgBytesPerSec;
	unsigned short	  nBlockAlign;
	unsigned short    wBitsPerSample;
	unsigned short    cbSize;
} WAVEFORMATEX;
#endif

#endif
