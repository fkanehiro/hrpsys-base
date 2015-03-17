// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "SampleComponent_impl.h"
#include "SampleComponent.h"

SampleComponent_impl::SampleComponent_impl()
{
}

SampleComponent_impl::~SampleComponent_impl()
{
}

void SampleComponent_impl::echo(const char *msg)
{
	std::cout << "SampleComponent: " << msg << std::endl;
	if (std::string(msg) == "up") {
		m_sample->resetOffset(-0.002);
	} else {
		m_sample->resetOffset(0.002);
	}
}

void SampleComponent_impl::sample(SampleComponent *i_sample)
{
    m_sample = i_sample;
} 

