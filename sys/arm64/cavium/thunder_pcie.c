/*-
 * Copyright (c) 2014 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Semihalf under
 * the sponsorship of the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/* PCIe root complex driver for Cavium Thunder SOC */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <machine/cpu.h>
#include <machine/bus.h>
#include <machine/fdt.h>
#include <dev/fdt/fdt_common.h>

/* Assembling ECAM Configuration Address */
#define PCIE_BUS_SHIFT	20
#define PCIE_SLOT_SHIFT 15
#define PCIE_FUNC_SHIFT 12
#define PCIE_BUS_MASK	0xFF
#define PCIE_SLOT_MASK	0x1F
#define PCIE_FUNC_MASK	0x07
#define PCIE_REG_MASK	0xFFF

#define PCIE_ADDR_OFFSET(bus, slot, func, reg)			\
	((((bus) & PCIE_BUS_MASK) << PCIE_BUS_SHIFT)	|	\
	(((slot) & PCIE_SLOT_MASK) << PCIE_SLOT_SHIFT)	|	\
	(((func) & PCIE_FUNC_MASK) << PCIE_FUNC_SHIFT)	|	\
	((reg) & PCIE_REG_MASK))

#define ECAM_COUNT		4
#define MAX_RANGES_TUPLES	3
#define MIN_RANGES_TUPLES	2

/* Forward prototypes */
static int thunder_pcie_probe(device_t dev);
static int thunder_pcie_attach(device_t dev);
static int parse_pci_mem_ranges(device_t dev);
static uint32_t thunder_pcie_read_config(device_t dev, uint32_t bus, uint32_t
devfn, uint32_t reg, uint32_t bytes);
static void thunder_pcie_write_config(device_t dev, uint32_t bus,
uint32_t devfn, uint32_t reg, uint32_t val, uint32_t bytes);

struct pcie_range {
	uint64_t	base;
	uint64_t	size;
};

struct thunder_pcie_softc {
	struct pcie_range	ranges[ECAM_COUNT][MAX_RANGES_TUPLES];
	struct rman		mem_rman;
	struct resource		*res;
	bus_space_tag_t		bst;
	bus_space_handle_t      bsh;
	device_t		dev;
};

static int
thunder_pcie_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "cavium,thunder-pcie")) {
		device_set_desc(dev, "Cavium Integrated PCI/PCI-E Controller");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
thunder_pcie_attach(device_t dev)
{
	int rid;
	struct thunder_pcie_softc *sc;
	int error;
	int ecam, tuple;
	uint64_t base,size;

	sc = device_get_softc(dev);

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not map memory\n");
		return (ENXIO);
	}

	sc->bst = rman_get_bustag(sc->res);
	sc->bsh = rman_get_bushandle(sc->res);

	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "PCIe Memory";

	/* Retrieve 'ranges' property from FDT */

	if (parse_pci_mem_ranges(dev))
		return (ENXIO);

	/* Initialize rman and allocate memory regions */

	error = rman_init(&sc->mem_rman);
	if (error) {
		device_printf(dev, "rman_init() failed. error = %d\n", error);
		return (error);
	}

	for (ecam = 0; ecam < ECAM_COUNT; ecam++) {
		for (tuple = 0; tuple < MAX_RANGES_TUPLES; tuple++) {
			base = sc->ranges[ecam][tuple].base; 
			size = sc->ranges[ecam][tuple].size; 
			if (base == 0 || size == 0)
				continue; /* empty range element */

			error = rman_manage_region(&sc->mem_rman, base, base + size - 1);
			if (error) {
				device_printf(dev, "rman_manage_region() failed. error = %d\n", error);
				rman_fini(&sc->mem_rman);
				return (error);
			}
		}
	}

	/* add stuff here */

	return (0); /* change to default bus attach */
}

static int
parse_pci_mem_ranges(device_t dev)
{
	struct thunder_pcie_softc *sc;
	phandle_t node;
	pcell_t pci_addr_cells, parent_addr_cells, size_cells;
	pcell_t *ranges_buf, *cell_ptr;
	int cells_count, tuples_count;
	int tuple;
	int ecam; /* pcie unit number */
	int rv;

	sc = device_get_softc(dev);
	ecam = device_get_unit(dev);
	node = ofw_bus_get_node(dev);

	if (fdt_addrsize_cells(node, &pci_addr_cells, &size_cells))
		return (ENXIO);

	parent_addr_cells = fdt_parent_addr_cells(node);
	if (parent_addr_cells != 2 || pci_addr_cells != 3 || size_cells != 2) {
		device_printf(dev, "Unexpected number of address or size cells in FDT");
		return (ENXIO);
	}

	cells_count = OF_getprop_alloc(node, "ranges", 
	    sizeof(pcell_t), (void **)&ranges_buf);
	if (cells_count == -1) {
		device_printf(dev, "Error parsing FDT 'ranges' property");
		return (ENXIO);
	}

	tuples_count = cells_count / 
	    (pci_addr_cells + parent_addr_cells + size_cells);
	if (tuples_count > MAX_RANGES_TUPLES || tuples_count < MIN_RANGES_TUPLES) {
		device_printf(dev, "Unexpected number of 'ranges' tuples in FDT");
		rv = ENXIO;
		goto out;
	}

	cell_ptr = ranges_buf;
	for (tuple = 0; tuple < tuples_count; tuple++) {
		cell_ptr += pci_addr_cells; /* move ptr to parent addr */
		sc->ranges[ecam][tuple].base = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += parent_addr_cells; /* move ptr to size cells*/
		sc->ranges[ecam][tuple].size = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += size_cells; /* move ptr to next tuple*/
	}
	for (; tuple < MAX_RANGES_TUPLES; tuple++) {
		/* zero-fill remaining tuples to mark empty elements in array */
		sc->ranges[ecam][tuple].base = 0;
		sc->ranges[ecam][tuple].size = 0;
	}

	rv = 0;
out:
	free(ranges_buf, M_OFWPROP);
	return (rv);
}

static uint32_t
thunder_pcie_read_config(device_t dev, uint32_t bus, uint32_t devfn,
    uint32_t reg, uint32_t bytes)
{
	uint64_t offset;
	uint32_t data;
	struct thunder_pcie_softc *sc;

	if (bus > 255 || devfn > 255 || reg > 4095)
		return (~0U);

	sc = device_get_softc(dev);
	offset = PCIE_ADDR_OFFSET(bus, devfn >> 3, devfn & 0x7, reg);

	switch (bytes) {
	case 1:
		data = bus_space_read_1(sc->bst, sc->bsh, offset);
		break;
	case 2:
		data = le16toh(bus_space_read_2(sc->bst, sc->bsh, offset));
		break;
	case 4:
		data = le32toh(bus_space_read_4(sc->bst, sc->bsh, offset));
		break;
	default:
		return (~0U);
	}

	return (data);
}

static void
thunder_pcie_write_config(device_t dev, uint32_t bus, uint32_t devfn,
    uint32_t reg, uint32_t val, uint32_t bytes)
{
	uint64_t offset;
	struct thunder_pcie_softc *sc;

	if (bus > 255 || devfn > 255 || reg > 4095)
		return;

	sc = device_get_softc(dev);
	offset = PCIE_ADDR_OFFSET(bus, devfn >> 3, devfn & 0x7, reg);

	switch (bytes) {
	case 1:
		bus_space_write_1(sc->bst, sc->bsh, offset, val);
		break;
	case 2:
		bus_space_write_2(sc->bst, sc->bsh, offset, htole16(val));
		break;
	case 4:
		bus_space_write_4(sc->bst, sc->bsh, offset, htole32(val));
		break;
	default:
		return;
	}

}

static device_method_t thunder_pcie_methods[] = {
	DEVMETHOD(device_probe,		thunder_pcie_probe),
	DEVMETHOD(device_attach,	thunder_pcie_attach),

	DEVMETHOD_END
};

static driver_t thunder_pcie_driver = {
	"pcie",
	thunder_pcie_methods,
	sizeof(struct thunder_pcie_softc),
};

static devclass_t thunder_pcie_devclass;

DRIVER_MODULE(thunder_pcie, simplebus, thunder_pcie_driver,
thunder_pcie_devclass, 0, 0);
DRIVER_MODULE(thunder_pcie, ofwbus, thunder_pcie_driver,
thunder_pcie_devclass, 0, 0);
