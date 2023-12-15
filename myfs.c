/*
*  myfs.c - Implementacao do sistema de arquivos MyFS
*
*  Autores: Arthur de Freitas Dornelas
			João Cotta Badaró
			Vinicius da Cruz Soranço
*  Projeto: Trabalho Pratico II - Sistemas Operacionais
*  Organizacao: Universidade Federal de Juiz de Fora
*  Departamento: Dep. Ciencia da Computacao
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "myfs.h"
#include "vfs.h"
#include "inode.h"
#include "util.h"

#define SB_FIRST_BLOCK_SECTOR (2 * sizeof(unsigned int) + sizeof(char))
#define SB_NUM_BLOCKS (3 * sizeof(unsigned int) + sizeof(char))
#define SB_FREE_SPACE_SECTOR (sizeof(unsigned int) + sizeof(char))
#define SB_BLOCKSIZE 0

typedef struct
{
	char filename[100];
	unsigned int inumber;
} LinkDir;

typedef struct // estrutura para arquivo
{
	Disk *disk;
	Inode *inode;
	unsigned int blocksize;
	unsigned int lastByteRead;
	const char *path;
	unsigned int fd;
} File;

typedef struct // estrutura para diretório
{
	Disk *disk;
	Inode *inode;
	unsigned int blocksize;
	unsigned int lastByteRead;
	const char *path;
	unsigned int fd;
} Directory;

typedef struct
{
	char filename[MAX_FILENAME_LENGTH];
	unsigned int inumber;
} DirectoryEntry;

// Declaracoes globais
FSInfo *fsInfo;
File *files[MAX_FDS] = {NULL};
Directory *directories[MAX_FDS] = {NULL};

int firstZeroBit(unsigned char byte)
{
	unsigned char mask = 1;

	for (int i = 0; i < sizeof(unsigned char); i++)
	{
		if ((mask & byte) == 0)
			return i;

		mask <<= (unsigned char)1;
	}

	return -1;
}

unsigned char setBitToOne(unsigned char byte, unsigned int bit)
{
	unsigned char mask = (unsigned char)1 << bit;
	return byte | mask;
}

unsigned char setBitToZero(unsigned char byte, unsigned int bit)
{
	unsigned char mask = ((unsigned char)1 << bit);
	mask = ~mask;
	return byte & mask;
}

unsigned int findFreeBlock(Disk *disk)
{
	unsigned char buffer[DISK_SECTORDATASIZE];
	if (diskReadSector(disk, 0, buffer) == -1)
		return -1;

	unsigned int sectorsPerBlock;
	char2ul(&buffer[SB_BLOCKSIZE], &sectorsPerBlock);
	sectorsPerBlock /= DISK_SECTORDATASIZE;

	unsigned int numBlocks;
	char2ul(&buffer[SB_NUM_BLOCKS], &numBlocks);

	unsigned int firstBlock;
	char2ul(&buffer[SB_FIRST_BLOCK_SECTOR], &firstBlock);

	unsigned int freeSpaceSector;
	char2ul(&buffer[SB_FREE_SPACE_SECTOR], &freeSpaceSector);

	unsigned int freeSpaceSize = firstBlock - freeSpaceSector;

	for (int i = freeSpaceSector; i < freeSpaceSector + freeSpaceSize; i++)
	{
		if (diskReadSector(disk, i, buffer) == -1)
			return -1;

		for (int j = 0; j < DISK_SECTORDATASIZE; j++)
		{
			int freeBit = firstZeroBit(buffer[j]);

			if (freeBit != -1)
			{
				unsigned int freeBlock = firstBlock +
										 (i - freeSpaceSector) * DISK_SECTORDATASIZE * 8 * sectorsPerBlock +
										 j * 8 * sectorsPerBlock +
										 freeBit * sectorsPerBlock;

				if ((freeBlock - firstBlock) / sectorsPerBlock >= numBlocks)
					return -1;

				buffer[j] = setBitToOne(buffer[j], freeBit);
				if (diskWriteSector(disk, i, buffer) == -1)
					return -1;

				return freeBlock;
			}
		}
	}

	return -1;
}

bool setBlockFree(Disk *d, unsigned int block)
{
	unsigned char buffer[DISK_SECTORDATASIZE];
	if (diskReadSector(d, 0, buffer) == -1)
		return false;

	unsigned int sectorsPerBlock;
	char2ul(&buffer[SB_BLOCKSIZE], &sectorsPerBlock);
	sectorsPerBlock /= DISK_SECTORDATASIZE;

	unsigned int numBlocks;
	char2ul(&buffer[SB_NUM_BLOCKS], &numBlocks);

	unsigned int firstBlock;
	char2ul(&buffer[SB_FIRST_BLOCK_SECTOR], &firstBlock);

	unsigned int freeSpaceStartSector;
	char2ul(&buffer[SB_FREE_SPACE_SECTOR], &freeSpaceStartSector);

	if ((block - firstBlock) / sectorsPerBlock >= numBlocks)
		return false;

	unsigned int blockFreeSpaceSector = ((block - firstBlock) / sectorsPerBlock) / (DISK_SECTORDATASIZE * 8);
	if (diskReadSector(d, blockFreeSpaceSector, buffer) == -1)
		return false;

	unsigned int blockFreeSpaceBit = ((block - firstBlock) / sectorsPerBlock) % (DISK_SECTORDATASIZE * 8);
	buffer[blockFreeSpaceBit / 8] = setBitToZero(buffer[blockFreeSpaceBit / 8], blockFreeSpaceBit % 8);

	if (diskWriteSector(d, blockFreeSpaceSector, buffer) == -1)
		return false;

	return true;
}

File *getFile(Disk *d, const char *path)
{
	for (int i = 0; i < MAX_FDS; i++)
	{
		if (files[i] != NULL && files[i]->disk == d && strcmp(files[i]->path, path) == 0)
		{
			return files[i];
		}
	}
	return NULL;
}
/////FIM - FUNCOES AUXILIARES/////

// Funcao para verificacao se o sistema de arquivos está ocioso, ou seja,
// se nao ha quisquer descritores de arquivos em uso atualmente. Retorna
// um positivo se ocioso ou, caso contrario, 0.
int myFSIsIdle(Disk *d)
{
	for (int i = 0; i < MAX_FDS; i++)
	{
		if (files[i] != NULL && diskGetId(d) == diskGetId(files[i]->disk))
		{
			return 0;
		}
	}
	return 1;
}

// Funcao para formatacao de um disco com o novo sistema de arquivos
// com tamanho de blocos igual a blockSize. Retorna o numero total de
// blocos disponiveis no disco, se formatado com sucesso. Caso contrario,
// retorna -1.
int myFSFormat(Disk *d, unsigned int blockSize)
{
	unsigned char superblock[DISK_SECTORDATASIZE] = {0};

	ul2char(blockSize, &superblock[SB_BLOCKSIZE]); // funçao para conversao de um unsigned int para um array de bytes

	unsigned int numInodes = (diskGetSize(d) / blockSize) / 8;

	unsigned int freeSpaceSector = inodeAreaBeginSector() + numInodes / inodeNumInodesPerSector();
	unsigned int freeSpaceSize = (diskGetSize(d) / blockSize) / (sizeof(unsigned char) * 8 * DISK_SECTORDATASIZE);

	ul2char(freeSpaceSector, &superblock[SB_FREE_SPACE_SECTOR]);

	unsigned int firstBlockSector = freeSpaceSector + freeSpaceSize;
	unsigned int numBlocks = (diskGetNumSectors(d) - firstBlockSector) / (blockSize / DISK_SECTORDATASIZE);

	ul2char(firstBlockSector, &superblock[SB_FIRST_BLOCK_SECTOR]);
	ul2char(numBlocks, &superblock[SB_NUM_BLOCKS]);

	if (diskWriteSector(d, 0, superblock) == -1)
		return -1;

	unsigned char freeSpace[DISK_SECTORDATASIZE] = {0};
	for (int i = 0; i < freeSpaceSize; i++)
	{
		if (diskWriteSector(d, freeSpaceSector + i, freeSpace) == -1)
		{
			return -1;
		}
	}

	return numBlocks > 0 ? numBlocks : -1;
}

File *MygetFile(Disk *d, const char *path)
{
	for (int i = 0; i < MAX_FDS; i++)
	{
		if (files[i] != NULL && files[i]->disk == d && strcmp(files[i]->path, path) == 0)
		{
			return files[i];
		}
	}
	return NULL;
}

// Funcao para abertura de um arquivo, a partir do caminho especificado
// em path, no disco montado especificado em d, no modo Read/Write,
// criando o arquivo se nao existir. Retorna um descritor de arquivo,
// em caso de sucesso. Retorna -1, caso contrario.
int myFSOpen(Disk *d, const char *path)
{
	File *file = getFile(d, path);
	int numInode;

	// arquivo não existe ,entao vamos criá-lo
	if (file == NULL)
	{
		Inode *inode = NULL;
		for (int i = 0; i < MAX_FDS; i++)
		{
			if (files[i] == NULL)
			{
				numInode = inodeFindFreeInode(1, d);
				inode = inodeCreate(numInode, d);
				break;
			}
		}

		if (inode == NULL)
			return -1;

		file = malloc(sizeof(File));
		file->disk = d;
		file->path = path;
		file->inode = inode;
		file->fd = inodeGetNumber(inode);
		files[file->fd - 1] = file;
	}

	return file->fd;
}

// Funcao para a leitura de um arquivo, a partir de um descritor de
// arquivo existente. Os dados lidos sao copiados para buf e terao
// tamanho maximo de nbytes. Retorna o numero de bytes efetivamente
// lidos em caso de sucesso ou -1, caso contrario.
int myFSRead(int fd, char *buf, unsigned int nbytes)
{
	if (fd < 0 || fd >= MAX_FDS)
		return -1;

	File *file = files[fd];
	if (file == NULL)
		return -1;

	unsigned int fileSize = inodeGetFileSize(file->inode);
	unsigned int bytesRead = 0;
	unsigned int currentInodeBlockNum = file->lastByteRead / file->blocksize;
	unsigned int offset = file->lastByteRead % file->blocksize;
	unsigned int currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNum);
	unsigned char diskBuffer[DISK_SECTORDATASIZE];

	while (bytesRead < nbytes &&
		   bytesRead + file->lastByteRead < fileSize &&
		   currentBlock > 0)
	{
		unsigned int sectorsPerBlock = file->blocksize / DISK_SECTORDATASIZE;
		unsigned int firstSector = offset / DISK_SECTORDATASIZE;
		unsigned int firstByteInSector = offset % DISK_SECTORDATASIZE;

		for (int i = firstSector; i < sectorsPerBlock && bytesRead < nbytes; i++)
		{
			if (diskReadSector(file->disk, currentBlock + i, diskBuffer) == -1)
				return -1;

			for (int j = firstByteInSector; j < DISK_SECTORDATASIZE &&
											bytesRead < nbytes &&
											bytesRead + file->lastByteRead < fileSize;
				 j++)
			{
				buf[bytesRead] = diskBuffer[j];
				bytesRead++;
			}

			firstByteInSector = 0;
		}

		offset = 0;
		currentInodeBlockNum++;
		currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNum);
	}

	file->lastByteRead += bytesRead;

	return bytesRead;
}

// Funcao para a escrita de um arquivo, a partir de um descritor de
// arquivo existente. Os dados de buf serao copiados para o disco e
// terao tamanho maximo de nbytes. Retorna o numero de bytes
// efetivamente escritos em caso de sucesso ou -1, caso contrario
int myFSWrite(int fd, const char *buf, unsigned int nbytes)
{
	if (fd <= 0 || fd > MAX_FDS)
		return -1;

	File *file = files[fd];
	if (!file)
		return -1;

	unsigned int fileSize = inodeGetFileSize(file->inode);
	unsigned int bytesWritten = 0;
	unsigned int currentInodeBlockNum = file->lastByteRead / file->blocksize;
	unsigned int offset = file->lastByteRead % file->blocksize;
	unsigned int currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNum);
	unsigned char diskBuffer[DISK_SECTORDATASIZE];

	while (bytesWritten < nbytes)
	{
		unsigned int sectorsPreBlock = file->blocksize / DISK_SECTORDATASIZE;
		unsigned int firstSector = offset / DISK_SECTORDATASIZE;
		unsigned int firstByteInSector = offset % DISK_SECTORDATASIZE;

		if (currentBlock == 0)
		{
			currentBlock = findFreeBlock(file->disk);

			if (currentBlock == -1)
				break;

			if (inodeAddBlock(file->inode, currentBlock) == -1)
			{
				setBlockFree(file->disk, currentBlock);
				break;
			}
		}

		for (int i = firstSector; i < sectorsPreBlock && bytesWritten < nbytes; i++)
		{
			if (diskReadSector(file->disk, currentBlock + i, diskBuffer) == -1)
				return -1;

			for (int j = firstByteInSector; j < DISK_SECTORDATASIZE && bytesWritten < nbytes; j++)
			{
				diskBuffer[j] = buf[bytesWritten];
				bytesWritten++;
			}

			if (diskWriteSector(file->disk, currentBlock + i, diskBuffer) == -1)
				return -1;

			firstByteInSector = 0;
		}

		offset = 0;
		currentInodeBlockNum++;
		currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNum);
	}

	file->lastByteRead += bytesWritten;
	if (file->lastByteRead >= fileSize)
	{
		inodeSetFileSize(file->inode, currentInodeBlockNum);
		inodeSave(file->inode);
	}
	return bytesWritten;
}

// Funcao para fechar um arquivo, a partir de um descritor de arquivo
// existente. Retorna 0 caso bem sucedido, ou -1 caso contrario
int myFSClose(int fd)
{
	if (fd <= 0 || fd > MAX_FDS)
		return -1;

	File *file = files[fd];
	if (!file)
		return -1;

	files[fd - 1] = NULL;
	free(file->inode);
	free(file);

	return 0;
}

// Funcao para abertura de um diretorio, a partir do caminho
// especificado em path, no disco indicado por d, no modo Read/Write,
// criando o diretorio se nao existir. Retorna um descritor de arquivo,
// em caso de sucesso. Retorna -1, caso contrario.
int myFSOpenDir(Disk *d, const char *path)
{

	if (path == NULL)
		return -1;

	if (strlen(path) == 0)
		return -1;

	Directory *dir = NULL;

	for (int i = 0; i < MAX_FDS; i++)
	{
		if (directories[i] == NULL)
		{
			dir = malloc(sizeof(Directory));
			if (dir == NULL)
			{
				return -1; // Falha na alocação de memória
			}

			// Encontrar um i-node livre para o diretório
			unsigned int inodeNumber = inodeFindFreeInode(1, d);
			if (inodeNumber == 0)
			{
				free(dir);
				return -1; // Não foi possível encontrar um i-node livre
			}

			// Criar o i-node para o diretório
			dir->inode = inodeCreate(inodeNumber, d);
			if (dir->inode == NULL)
			{
				free(dir);
				return -1; // Falha ao criar o i-node
			}

			dir->disk = d;
			dir->path = path;
			dir->blocksize = SB_BLOCKSIZE; // Substitua pelo tamanho real do bloco
			dir->lastByteRead = 0;

			// Armazenar o diretório aberto na lista global
			directories[i] = dir;

			return i + 1; // Retorna o descritor de arquivo (índice + 1)
		}
	}
	return -1; // se não achou um lugar livre para o diretório
}

// Função auxiliar para obter a próxima entrada de diretório
int getNextDirEntry(Disk *disk, Inode *dirInode, unsigned int *cursor, DirectoryEntry *entry)
{
	unsigned int dirSize = inodeGetFileSize(dirInode);

	// Verifica se já atingiu o fim do diretório
	if (*cursor >= dirSize)
	{
		return 0; // Fim do diretório
	}

	// Calcula o número do bloco e a posição dentro do bloco
	unsigned int blockNum = *cursor / DISK_SECTORDATASIZE;
	unsigned int blockOffset = *cursor % DISK_SECTORDATASIZE;

	// Lê o bloco do diretório
	unsigned char blockBuffer[DISK_SECTORDATASIZE];
	if (diskReadSector(disk, inodeGetBlockAddr(dirInode, blockNum), blockBuffer) == -1)
	{
		return -1; // Falha na leitura
	}

	// Copia a entrada de diretório do bloco para a estrutura entry
	memcpy(entry, blockBuffer + blockOffset, sizeof(DirectoryEntry));

	// Atualiza o cursor
	*cursor += sizeof(DirectoryEntry);

	return 1; // Entrada de diretório lida com sucesso
}

// Funcao para a leitura de um diretorio, identificado por um descritor
// de arquivo existente. Os dados lidos correspondem a uma entrada de
// diretorio na posicao atual do cursor no diretorio. O nome da entrada
// e' copiado para filename, como uma string terminada em \0 (max 255+1).
// O numero do inode correspondente 'a entrada e' copiado para inumber.
// Retorna 1 se uma entrada foi lida, 0 se fim de diretorio ou -1 caso
// mal sucedido
int myFSReadDir(int fd, char *filename, unsigned int *inumber)
{
	// Verifica se o descritor de arquivo é válido
	if (fd <= 0 || fd > MAX_FDS)
	{
		return -1; // Descritor de arquivo inválido
	}

	File *dirFile = files[fd - 1];
	if (dirFile == NULL)
	{
		return -1; // Diretório não encontrado
	}

	// Obtém a próxima entrada de diretório
	DirectoryEntry entry;
	int result = getNextDirEntry(dirFile->disk, dirFile->inode, &dirFile->lastByteRead, &entry);

	if (result == 1)
	{
		// Entrada de diretório lida com sucesso, copia os dados para os parâmetros de saída
		strncpy(filename, entry.filename, MAX_FILENAME_LENGTH);
		*inumber = entry.inumber;
	}

	return result; // Retorna o resultado da leitura do diretório
}

// Funcao para adicionar uma entrada a um diretorio, identificado por um
// descritor de arquivo existente. A nova entrada tera' o nome indicado
// por filename e apontara' para o numero de i-node indicado por inumber.
// Retorna 0 caso bem sucedido, ou -1 caso contrario.
int myFSLink(int fd, const char *filename, unsigned int inumber)
{
	// Verifica se o descritor de arquivo é válido
	if (fd <= 0 || fd > MAX_FDS)
	{
		return -1; // Descritor de arquivo inválido
	}

	File *dirFile = files[fd - 1];
	if (dirFile == NULL)
	{
		return -1; // Diretório não encontrado
	}

	// Verifica se o nome do arquivo é válido
	if (strlen(filename) == 0 || strlen(filename) > MAX_FILENAME_LENGTH)
	{
		return -1; // Nome de arquivo inválido
	}

	// Cria uma entrada de diretório com os dados fornecidos
	DirectoryEntry newEntry;
	strncpy(newEntry.filename, filename, MAX_FILENAME_LENGTH);
	newEntry.inumber = inumber;

	// Calcula o número do bloco e a posição dentro do bloco
	unsigned int blockNum = dirFile->lastByteRead / DISK_SECTORDATASIZE;
	unsigned int blockOffset = dirFile->lastByteRead % DISK_SECTORDATASIZE;

	// Lê o bloco do diretório
	unsigned char blockBuffer[DISK_SECTORDATASIZE];
	if (diskReadSector(dirFile->disk, inodeGetBlockAddr(dirFile->inode, blockNum), blockBuffer) == -1)
	{
		return -1; // Falha na leitura
	}

	// Verifica se há espaço suficiente no bloco para a nova entrada
	if (blockOffset + sizeof(DirectoryEntry) > DISK_SECTORDATASIZE)
	{
		return -1; // Não há espaço suficiente no bloco
	}

	// Copia a nova entrada para o bloco do diretório
	memcpy(blockBuffer + blockOffset, &newEntry, sizeof(DirectoryEntry));

	// Escreve o bloco atualizado de volta no disco
	if (diskWriteSector(dirFile->disk, inodeGetBlockAddr(dirFile->inode, blockNum), blockBuffer) == -1)
	{
		return -1; // Falha na escrita
	}

	// Atualiza o cursor
	dirFile->lastByteRead += sizeof(DirectoryEntry);

	return 0; // Link criado com sucesso
}

// Funcao para remover uma entrada existente em um diretorio,
// identificado por um descritor de arquivo existente. A entrada e'
// identificada pelo nome indicado em filename. Retorna 0 caso bem
// sucedido, ou -1 caso contrario.
int myFSUnlink(int fd, const char *filename)
{
	return -1;
}

// Funcao para fechar um diretorio, identificado por um descritor de
// arquivo existente. Retorna 0 caso bem sucedido, ou -1 caso contrario.
int myFSCloseDir(int fd)
{
	return -1;
}

// Funcao para instalar seu sistema de arquivos no S.O., registrando-o junto
// ao virtual FS (vfs). Retorna um identificador unico (slot), caso
// o sistema de arquivos tenha sido registrado com sucesso.
// Caso contrario, retorna -1
int installMyFS(void) // feito(vinicius)
{
	fsInfo = malloc(sizeof(FSInfo));
	// fsInfo->fsname = "DTFS";
	fsInfo->fsid = (char)vfsRegisterFS(fsInfo);
	fsInfo->isidleFn = myFSIsIdle;
	fsInfo->formatFn = myFSFormat;
	fsInfo->openFn = myFSOpen;
	fsInfo->readFn = myFSRead;
	fsInfo->writeFn = myFSWrite;
	fsInfo->closeFn = myFSClose;
	// fsInfo->linkFn = myFSLink;
}