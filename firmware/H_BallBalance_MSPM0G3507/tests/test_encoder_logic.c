#include <assert.h>
#include <stdint.h>

#include "encoder_logic.h"

static void test_forward_sequence_counts_four_edges(void)
{
    const uint8_t states[] = {0U, 1U, 3U, 2U, 0U};
    int32_t total = 0;

    for (uint32_t i = 1U; i < (sizeof(states) / sizeof(states[0])); i++)
    {
        uint8_t valid = 0U;
        total += EncoderLogic_Transition(states[i - 1U], states[i], &valid);
        assert(valid == 1U);
    }

    assert(total == 4);
}

static void test_reverse_sequence_counts_minus_four_edges(void)
{
    const uint8_t states[] = {0U, 2U, 3U, 1U, 0U};
    int32_t total = 0;

    for (uint32_t i = 1U; i < (sizeof(states) / sizeof(states[0])); i++)
    {
        uint8_t valid = 0U;
        total += EncoderLogic_Transition(states[i - 1U], states[i], &valid);
        assert(valid == 1U);
    }

    assert(total == -4);
}

static void test_invalid_two_bit_jump_is_rejected(void)
{
    uint8_t valid = 1U;

    assert(EncoderLogic_Transition(0U, 3U, &valid) == 0);
    assert(valid == 0U);
}

static void test_same_state_is_valid_without_a_count(void)
{
    uint8_t valid = 0U;

    assert(EncoderLogic_Transition(2U, 2U, &valid) == 0);
    assert(valid == 1U);
}

static void test_qei_delta_handles_both_wrap_directions(void)
{
    assert(EncoderLogic_Delta16(2U, 65534U) == 4);
    assert(EncoderLogic_Delta16(65534U, 2U) == -4);
}

int main(void)
{
    test_forward_sequence_counts_four_edges();
    test_reverse_sequence_counts_minus_four_edges();
    test_invalid_two_bit_jump_is_rejected();
    test_same_state_is_valid_without_a_count();
    test_qei_delta_handles_both_wrap_directions();
    return 0;
}
